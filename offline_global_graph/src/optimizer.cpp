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
#include <cmath>
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
  if (huber_k <= 0.0) {
    return poseNoise(translation_sigma_m, rotation_sigma_rad);
  }
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
    poseNoise(translation_sigma_m, rotation_sigma_rad));
}

const TagPrior *findPrior(const std::vector<TagPrior> &priors, int tag_id)
{
  for (const auto &prior : priors) {
    if (prior.tag_id == tag_id) {
      return &prior;
    }
  }
  return nullptr;
}

double translationDistance(const gtsam::Pose3 &a, const gtsam::Pose3 &b)
{
  const gtsam::Point3 delta = a.translation() - b.translation();
  return delta.norm();
}

const TagPrior *findAnchorPrior(const std::vector<TagPrior> &priors)
{
  if (priors.empty()) {
    return nullptr;
  }

  for (const auto &prior : priors) {
    if (prior.anchor) {
      return &prior;
    }
  }

  return &priors.front();
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

  const TagPrior *anchor_prior = findAnchorPrior(session.tag_priors);
  if (anchor_prior != nullptr) {
    const TagObservation *seed_observation = nullptr;
    size_t seed_index = 0;
    for (size_t i = 0; i < session.keyframes.size() && seed_observation == nullptr; ++i) {
      const auto &keyframe = session.keyframes[i];
      for (const auto &observation : session.tag_observations) {
        if (observation.kf_id == keyframe.kf_id && observation.tag_id == anchor_prior->tag_id) {
          seed_observation = &observation;
          seed_index = i;
          break;
        }
      }
    }

    if (seed_observation != nullptr) {
      keyframe_initials[seed_observation->kf_id] =
        anchor_prior->world_T_tag.compose(seed_observation->body_T_tag.inverse());

      for (size_t i = seed_index + 1; i < session.keyframes.size(); ++i) {
        const auto &previous = session.keyframes[i - 1];
        const auto &current = session.keyframes[i];
        const gtsam::Pose3 measurement = betweenMeasurementForAdjacentKeyframes(previous, current);
        keyframe_initials[current.kf_id] = keyframe_initials.at(previous.kf_id).compose(measurement);
      }

      for (size_t i = seed_index; i > 0; --i) {
        const auto &previous = session.keyframes[i - 1];
        const auto &current = session.keyframes[i];
        const gtsam::Pose3 measurement = betweenMeasurementForAdjacentKeyframes(previous, current);
        keyframe_initials[previous.kf_id] = keyframe_initials.at(current.kf_id).compose(measurement.inverse());
      }
    }
  }

  for (const auto &keyframe : session.keyframes) {
    initial_values.insert(xKey(keyframe.kf_id), keyframe_initials.at(keyframe.kf_id));
  }

  const auto between_noise = poseNoise(
    config.between_translation_sigma_m, config.between_rotation_sigma_rad);
  for (size_t i = 1; i < session.keyframes.size(); ++i) {
    const auto &previous = session.keyframes[i - 1];
    const auto &current = session.keyframes[i];
    const gtsam::Pose3 measurement = betweenMeasurementForAdjacentKeyframes(previous, current);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      xKey(previous.kf_id), xKey(current.kf_id), measurement, between_noise));
    ++result.between_factor_count;
  }

  for (const auto &observation : session.tag_observations) {
    const auto keyframe_it = keyframe_initials.find(observation.kf_id);
    if (keyframe_it == keyframe_initials.end()) {
      continue;
    }

    const gtsam::Key tag_key = tKey(observation.tag_id);
    if (!initial_values.exists(tag_key)) {
      const TagPrior *prior = findPrior(session.tag_priors, observation.tag_id);
      if (prior != nullptr) {
        initial_values.insert(tag_key, prior->world_T_tag);
      } else {
        initial_values.insert(tag_key, keyframe_it->second.compose(observation.body_T_tag));
      }
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

  bool anchor_assigned = false;
  for (size_t i = 0; i < session.tag_priors.size(); ++i) {
    const auto &prior = session.tag_priors[i];
    const bool use_anchor_noise =
      config.anchor_all_tag_priors || prior.anchor || (!anchor_assigned && i == 0);
    const gtsam::Key tag_key = tKey(prior.tag_id);
    if (!initial_values.exists(tag_key)) {
      initial_values.insert(tag_key, prior.world_T_tag);
    }

    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      tag_key,
      prior.world_T_tag,
      poseNoise(
        use_anchor_noise ?
          config.anchor_translation_sigma_m :
          (prior.has_custom_sigmas ? prior.translation_sigma_m : config.soft_prior_translation_sigma_m),
        use_anchor_noise ?
          config.anchor_rotation_sigma_rad :
          (prior.has_custom_sigmas ? prior.rotation_sigma_rad : config.soft_prior_rotation_sigma_rad))));
    ++result.prior_factor_count;

    if (use_anchor_noise && !anchor_assigned) {
      anchor_assigned = true;
      result.anchor_strategy = prior.anchor ?
        ("tag_prior:" + std::to_string(prior.tag_id)) :
        ("first_tag_prior_fallback:" + std::to_string(prior.tag_id));
    }
  }

  if (!anchor_assigned) {
    const auto &first_keyframe = session.keyframes.front();
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      xKey(first_keyframe.kf_id),
      first_keyframe.initial_pose_wb,
      poseNoise(
        config.anchor_translation_sigma_m,
        config.anchor_rotation_sigma_rad)));
    ++result.prior_factor_count;
    result.anchor_strategy = "first_keyframe_fallback:" + std::to_string(first_keyframe.kf_id);
  }

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
  for (const auto &prior : session.tag_priors) {
    const gtsam::Key tag_key = tKey(prior.tag_id);
    if (optimized.exists(tag_key)) {
      sorted_tags.emplace(prior.tag_id, optimized.at<gtsam::Pose3>(tag_key));
    }
  }
  for (const auto &observation : session.tag_observations) {
    const gtsam::Key tag_key = tKey(observation.tag_id);
    if (optimized.exists(tag_key)) {
      sorted_tags.emplace(observation.tag_id, optimized.at<gtsam::Pose3>(tag_key));
    }
  }
  for (const auto &entry : sorted_tags) {
    result.optimized_tags.emplace_back(entry.first, entry.second);
  }

  if (config.enforce_max_tag_translation_deviation) {
    for (const auto &prior : session.tag_priors) {
      const gtsam::Key tag_key = tKey(prior.tag_id);
      if (!optimized.exists(tag_key)) {
        continue;
      }

      const double deviation_m =
        translationDistance(optimized.at<gtsam::Pose3>(tag_key), prior.world_T_tag);
      if (deviation_m > config.max_tag_translation_deviation_m) {
        std::ostringstream oss;
        oss << "optimized tag " << prior.tag_id
            << " moved " << deviation_m << " m from its prior, exceeding the configured limit of "
            << config.max_tag_translation_deviation_m << " m";
        throw std::runtime_error(oss.str());
      }
    }
  }

  return result;
}

}  // namespace offline_global_graph
