#include "offline_global_graph/optimizer.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace offline_global_graph
{

namespace
{

gtsam::Key xKey(uint64_t kf_id)
{
  return gtsam::Symbol('x', kf_id);
}

gtsam::Key tKey(int tag_id)
{
  return gtsam::Symbol('t', static_cast<uint64_t>(tag_id));
}

gtsam::Key lKey(uint32_t track_id)
{
  return gtsam::Symbol('l', static_cast<uint64_t>(track_id));
}

gtsam::SharedNoiseModel poseNoise(double translation_sigma_m, double rotation_sigma_rad)
{
  const gtsam::Vector6 sigmas =
    (gtsam::Vector6() <<
      rotation_sigma_rad, rotation_sigma_rad, rotation_sigma_rad,
      translation_sigma_m, translation_sigma_m, translation_sigma_m).finished();
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::SharedNoiseModel robustPoseNoise(
  double translation_sigma_m,
  double rotation_sigma_rad,
  double huber_k)
{
  const auto base_noise = poseNoise(translation_sigma_m, rotation_sigma_rad);
  if (huber_k <= 0.0) {
    return base_noise;
  }
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
    base_noise);
}

gtsam::SharedNoiseModel maybeHuberize(
  const gtsam::SharedNoiseModel &base_noise,
  double huber_k)
{
  if (huber_k <= 0.0) {
    return base_noise;
  }
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
    base_noise);
}

double clamp01(double x)
{
  return std::clamp(x, 0.0, 1.0);
}

double normalizedFloorScore(double value, double min_acceptable)
{
  if (min_acceptable <= 0.0) {
    return 1.0;
  }
  return clamp01(value / min_acceptable);
}

double normalizedCeilingScore(double value, double max_acceptable)
{
  if (value < 0.0 || max_acceptable <= 0.0) {
    return 1.0;
  }
  return clamp01(max_acceptable / std::max(value, 1e-9));
}

double betweenIntervalQuality(
  const KeyframeRecord::IntervalHealth &health,
  const OptimizerConfig &config)
{
  if (health.num_frames == 0) {
    return 1.0;
  }

  const double num_frames = static_cast<double>(health.num_frames);
  const double pose_valid_fraction =
    static_cast<double>(health.num_pose_valid_frames) / num_frames;
  const double degraded_fraction =
    static_cast<double>(health.num_degraded_frames) / num_frames;
  const double lost_fraction =
    static_cast<double>(health.num_lost_frames) / num_frames;

  const double pose_valid_score = normalizedFloorScore(
    pose_valid_fraction, config.between_health_min_pose_valid_fraction);
  const double retention_score = normalizedFloorScore(
    health.min_track_retention, config.between_health_min_track_retention);
  const double pnp_score = normalizedFloorScore(
    health.mean_pnp_inlier_ratio, config.between_health_min_pnp_inlier_ratio);
  const double coverage_score = normalizedFloorScore(
    health.min_track_coverage, config.between_health_min_track_coverage);
  const double rmse_score = normalizedCeilingScore(
    health.max_pnp_reproj_rmse_px, config.between_health_max_pnp_reproj_rmse_px);
  const double state_score = clamp01(1.0 - std::max(degraded_fraction, lost_fraction));

  return std::min({
    pose_valid_score,
    retention_score,
    pnp_score,
    coverage_score,
    rmse_score,
    state_score});
}

bool shouldSkipBetweenInterval(
  const KeyframeRecord::IntervalHealth &health,
  const OptimizerConfig &config,
  double quality)
{
  if (health.num_frames == 0) {
    return false;
  }

  if (quality >= config.between_health_skip_quality) {
    return false;
  }

  return health.num_pose_valid_frames == 0;
}

gtsam::SharedNoiseModel optimizerPosePriorNoise(
  const KeyframeRecord &keyframe,
  const OptimizerConfig &config)
{
  gtsam::Matrix66 covariance = gtsam::Matrix66::Zero();
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      covariance(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
        keyframe.pose_wb_covariance[r * 6 + c];
    }
  }

  covariance = 0.5 * (covariance + covariance.transpose());
  covariance *= std::max(1.0, config.optimizer_pose_prior_covariance_scale);

  const double min_rot_var =
    config.optimizer_pose_prior_min_rotation_sigma_rad *
    config.optimizer_pose_prior_min_rotation_sigma_rad;
  const double min_trans_var =
    config.optimizer_pose_prior_min_translation_sigma_m *
    config.optimizer_pose_prior_min_translation_sigma_m;

  for (size_t i = 0; i < 3; ++i) {
    covariance(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) =
      std::max(covariance(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)), min_rot_var);
  }
  for (size_t i = 3; i < 6; ++i) {
    covariance(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) =
      std::max(covariance(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)), min_trans_var);
  }

  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      const double value = covariance(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
      if (!std::isfinite(value)) {
        throw std::runtime_error("optimizer pose covariance contains non-finite values");
      }
    }
  }

  return maybeHuberize(
    gtsam::noiseModel::Gaussian::Covariance(covariance),
    config.optimizer_pose_prior_huber_k);
}

gtsam::Pose3 betweenMeasurementForAdjacentKeyframes(
  const KeyframeRecord &previous,
  const KeyframeRecord &current)
{
  if (current.between_pose_prev_curr_body.has_value()) {
    return *current.between_pose_prev_curr_body;
  }
  return previous.initial_pose_wb.between(current.initial_pose_wb);
}

}  // namespace

OptimizationResult optimizeSession(const SessionData &session, const OptimizerConfig &config)
{
  if (session.keyframes.empty()) {
    throw std::runtime_error("cannot optimize an empty session");
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  OptimizationResult result;

  std::unordered_map<uint64_t, gtsam::Pose3> keyframe_initials;
  keyframe_initials.reserve(session.keyframes.size());
  for (const auto &keyframe : session.keyframes) {
    keyframe_initials.emplace(keyframe.kf_id, keyframe.initial_pose_wb);
  }

  for (const auto &keyframe : session.keyframes) {
    initial_values.insert(xKey(keyframe.kf_id), keyframe_initials.at(keyframe.kf_id));
  }

  for (size_t i = 1; i < session.keyframes.size(); ++i) {
    const auto &previous = session.keyframes[i - 1];
    const auto &current = session.keyframes[i];
    const gtsam::Pose3 measurement = betweenMeasurementForAdjacentKeyframes(previous, current);
    double sigma_scale = 1.0;
    bool skip_between = false;
    if (config.use_interval_health_for_between) {
      const double quality = betweenIntervalQuality(current.interval_health, config);
      sigma_scale =
        1.0 + (1.0 - quality) * std::max(0.0, config.between_health_max_sigma_scale - 1.0);
      skip_between = shouldSkipBetweenInterval(current.interval_health, config, quality);
      if (skip_between) {
        ++result.between_factor_skipped_count;
      } else if (sigma_scale > 1.01) {
        ++result.between_factor_inflated_count;
      }
    }
    if (skip_between) {
      continue;
    }
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      xKey(previous.kf_id),
      xKey(current.kf_id),
      measurement,
      poseNoise(
        config.between_translation_sigma_m * sigma_scale,
        config.between_rotation_sigma_rad * sigma_scale)));
    ++result.between_factor_count;
  }

  if (config.use_optimizer_pose_priors) {
    for (const auto &keyframe : session.keyframes) {
      if (!keyframe.has_pose_wb_covariance) {
        continue;
      }
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        xKey(keyframe.kf_id),
        keyframe.optimized_pose_wb,
        optimizerPosePriorNoise(keyframe, config)));
      ++result.prior_factor_count;
      ++result.optimizer_pose_prior_count;
    }
  }

  for (const auto &observation : session.tag_observations) {
    const auto keyframe_it = keyframe_initials.find(observation.kf_id);
    if (keyframe_it == keyframe_initials.end()) {
      continue;
    }

    const gtsam::Key tag_key = tKey(observation.tag_id);
    if (!initial_values.exists(tag_key)) {
      initial_values.insert(tag_key, keyframe_it->second.compose(observation.body_T_tag));
    }

    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      xKey(observation.kf_id),
      tag_key,
      observation.body_T_tag,
      robustPoseNoise(
        config.tag_translation_sigma_m,
        config.tag_rotation_sigma_rad,
        config.tag_observation_huber_k)));
    ++result.tag_observation_factor_count;
  }

  const auto &first_keyframe = session.keyframes.front();
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    xKey(first_keyframe.kf_id),
    first_keyframe.initial_pose_wb,
    poseNoise(
      config.anchor_translation_sigma_m,
      config.anchor_rotation_sigma_rad)));
  ++result.prior_factor_count;
  result.anchor_strategy = "first_keyframe_fallback:" + std::to_string(first_keyframe.kf_id);

  if (config.use_visual_factors) {
    if (session.camera.fx <= 0.0 || session.camera.fy <= 0.0) {
      throw std::runtime_error("visual factors requested but session camera intrinsics are unavailable");
    }

    using ProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;
    const auto visual_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(config.visual_huber_k),
      gtsam::noiseModel::Isotropic::Sigma(2, config.visual_sigma_px));
    auto calibration = std::make_shared<gtsam::Cal3_S2>(
      session.camera.fx,
      session.camera.fy,
      0.0,
      session.camera.cx,
      session.camera.cy);

    struct TrackRef
    {
      const KeyframeRecord *keyframe{nullptr};
      const KeyframeRecord::TrackObservation *observation{nullptr};
    };

    std::unordered_map<uint32_t, std::vector<TrackRef>> observations_by_track;
    observations_by_track.reserve(session.keyframes.size() * 16);
    for (const auto &keyframe : session.keyframes) {
      for (const auto &observation : keyframe.track_observations) {
        observations_by_track[observation.track_id].push_back(TrackRef{&keyframe, &observation});
      }
    }
    for (const auto &entry : observations_by_track) {
      const uint32_t track_id = entry.first;
      const auto &track_refs = entry.second;
      if (static_cast<int>(track_refs.size()) < config.min_track_observations) {
        continue;
      }

      std::optional<gtsam::Point3> initial_landmark;
      for (const auto &track_ref : track_refs) {
        if (!track_ref.observation->has_right) {
          continue;
        }

        const double disparity_px =
          static_cast<double>(track_ref.observation->u_l) -
          static_cast<double>(track_ref.observation->u_r);
        if (!std::isfinite(disparity_px) || disparity_px <= 1e-3) {
          continue;
        }

        const double z =
          (session.camera.fx * config.stereo_baseline_m) / disparity_px;
        if (!std::isfinite(z) || z < config.min_depth_m || z > config.max_depth_m) {
          continue;
        }

        const double x = (static_cast<double>(track_ref.observation->u_l) - session.camera.cx) * z / session.camera.fx;
        const double y = (static_cast<double>(track_ref.observation->v_l) - session.camera.cy) * z / session.camera.fy;
        const gtsam::Point3 point_in_camera(x, y, z);

        const auto pose_it = keyframe_initials.find(track_ref.keyframe->kf_id);
        if (pose_it == keyframe_initials.end()) {
          continue;
        }
        const gtsam::Pose3 world_T_camera = pose_it->second.compose(config.body_T_camera);
        initial_landmark = world_T_camera.transformFrom(point_in_camera);
        break;
      }

      if (!initial_landmark.has_value()) {
        continue;
      }

      const gtsam::Key landmark_key = lKey(track_id);
      if (!initial_values.exists(landmark_key)) {
        initial_values.insert(landmark_key, *initial_landmark);
        ++result.landmark_count;
      }

      for (const auto &track_ref : track_refs) {
        graph.add(ProjectionFactor(
          gtsam::Point2(track_ref.observation->u_l, track_ref.observation->v_l),
          visual_noise,
          xKey(track_ref.keyframe->kf_id),
          landmark_key,
          calibration,
          config.body_T_camera));
        ++result.visual_factor_count;
      }
    }
  }

  result.initial_error = graph.error(initial_values);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);
  const gtsam::Values optimized = optimizer.optimize();

  result.final_error = graph.error(optimized);

  for (const auto &keyframe : session.keyframes) {
    result.optimized_keyframes.emplace_back(
      keyframe.kf_id, optimized.at<gtsam::Pose3>(xKey(keyframe.kf_id)));
  }

  std::map<int, gtsam::Pose3> sorted_tags;
  for (const auto &observation : session.tag_observations) {
    const gtsam::Key tag_key = tKey(observation.tag_id);
    if (optimized.exists(tag_key)) {
      sorted_tags.emplace(observation.tag_id, optimized.at<gtsam::Pose3>(tag_key));
    }
  }
  for (const auto &entry : sorted_tags) {
    result.optimized_tags.emplace_back(entry.first, entry.second);
  }

  return result;
}

}  // namespace offline_global_graph
