#include "offline_global_graph/optimizer.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <map>
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

double computeBetweenIntervalQuality(
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

  const double pose_valid_score =
    normalizedFloorScore(pose_valid_fraction, config.between_health_min_pose_valid_fraction);
  const double retention_score =
    normalizedFloorScore(health.min_track_retention, config.between_health_min_track_retention);
  const double pnp_score =
    normalizedFloorScore(health.mean_pnp_inlier_ratio, config.between_health_min_pnp_inlier_ratio);
  const double coverage_score =
    normalizedFloorScore(health.min_track_coverage, config.between_health_min_track_coverage);
  const double rmse_score =
    normalizedCeilingScore(health.max_pnp_reproj_rmse_px, config.between_health_max_pnp_reproj_rmse_px);
  const double state_score = clamp01(1.0 - std::max(degraded_fraction, lost_fraction));

  return std::min({
    pose_valid_score,
    retention_score,
    pnp_score,
    coverage_score,
    rmse_score,
    state_score});
}

bool isLowQualityBetweenInterval(
  const KeyframeRecord::IntervalHealth &health,
  const OptimizerConfig &config,
  double interval_quality)
{
  if (health.num_frames == 0) {
    return false;
  }
  return interval_quality < config.between_health_skip_quality;
}

bool isDistantTagObservation(
  const TagObservation &observation,
  const OptimizerConfig &config)
{
  if (!config.reject_distant_tag_observations || config.max_tag_observation_distance_m <= 0.0) {
    return false;
  }

  const double range_m = observation.body_T_tag.translation().norm();
  return std::isfinite(range_m) && range_m > config.max_tag_observation_distance_m;
}

bool hasTooMuchHamming(
  const TagObservation &observation,
  const OptimizerConfig &config)
{
  if (!config.reject_tag_observations_with_hamming) {
    return false;
  }
  return observation.hamming > config.max_tag_hamming;
}

bool hasTooLowDecisionMargin(
  const TagObservation &observation,
  const OptimizerConfig &config)
{
  if (!config.reject_low_margin_tag_observations) {
    return false;
  }
  return observation.decision_margin < config.min_tag_decision_margin;
}

bool isObliqueTagObservation(
  const TagObservation &observation,
  const OptimizerConfig &config)
{
  if (!config.reject_oblique_tag_observations) {
    return false;
  }

  const gtsam::Point3 body_p_tag = observation.body_T_tag.translation();
  const double range_m = body_p_tag.norm();
  if (!std::isfinite(range_m) || range_m <= 1e-6) {
    return false;
  }

  constexpr double kDegToRad = 0.0174532925199432957692;
  constexpr double kHalfPiRad = 1.57079632679489661923;
  const double max_angle_rad = std::clamp(
    config.max_tag_oblique_angle_deg * kDegToRad,
    0.0,
    kHalfPiRad);
  const double min_abs_cosine = std::cos(max_angle_rad);

  const gtsam::Point3 tag_normal_body =
    observation.body_T_tag.rotation().rotate(gtsam::Point3(0.0, 0.0, 1.0));
  const double normal_norm = tag_normal_body.norm();
  if (!std::isfinite(normal_norm) || normal_norm <= 1e-6) {
    return false;
  }

  const double abs_cosine =
    std::abs(tag_normal_body.dot(-body_p_tag) / (normal_norm * range_m));
  return std::isfinite(abs_cosine) && abs_cosine < min_abs_cosine;
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
    keyframe_initials.emplace(keyframe.kf_id, keyframe.optimized_pose_wb);
    initial_values.insert(xKey(keyframe.kf_id), keyframe.optimized_pose_wb);
  }

  for (size_t i = 1; i < session.keyframes.size(); ++i) {
    const auto &previous = session.keyframes[i - 1];
    const auto &current = session.keyframes[i];
    gtsam::Pose3 between_measurement = previous.optimized_pose_wb.between(current.optimized_pose_wb);
    if (current.has_vo_between_measurement) {
      between_measurement = current.vo_between_prev_curr_body;
    }

    double sigma_scale = 1.0;
    double interval_quality = 1.0;
    bool low_quality_between = false;
    if (config.use_interval_health_for_between) {
      interval_quality = computeBetweenIntervalQuality(current.interval_health, config);
      sigma_scale = 1.0 +
        (1.0 - interval_quality) * std::max(0.0, config.between_health_max_sigma_scale - 1.0);
      low_quality_between = isLowQualityBetweenInterval(
        current.interval_health, config, interval_quality);
      result.min_between_interval_quality = std::min(result.min_between_interval_quality, interval_quality);
    }

    if (low_quality_between) {
      ++result.between_factor_low_quality_count;
      sigma_scale = std::max(sigma_scale, std::max(1.0, config.between_health_max_sigma_scale));
    }

    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      xKey(previous.kf_id),
      xKey(current.kf_id),
      between_measurement,
      poseNoise(
        config.between_translation_sigma_m * sigma_scale,
        config.between_rotation_sigma_rad * sigma_scale)));
    ++result.between_factor_count;
  }

  for (const auto &observation : session.tag_observations) {
    const auto keyframe_it = keyframe_initials.find(observation.kf_id);
    if (keyframe_it == keyframe_initials.end()) {
      continue;
    }

    if (isDistantTagObservation(observation, config)) {
      ++result.tag_observation_skipped_distance_count;
      continue;
    }

    if (hasTooMuchHamming(observation, config)) {
      ++result.tag_observation_skipped_hamming_count;
      continue;
    }

    if (hasTooLowDecisionMargin(observation, config)) {
      ++result.tag_observation_skipped_low_margin_count;
      continue;
    }

    if (isObliqueTagObservation(observation, config)) {
      ++result.tag_observation_skipped_oblique_count;
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

  if (config.use_tag_anchor_prior) {
    const gtsam::Key tag_key = tKey(config.tag_anchor_id);
    if (!initial_values.exists(tag_key)) {
      throw std::runtime_error(
        "requested tag anchor id " + std::to_string(config.tag_anchor_id) +
        " was not observed after gating");
    }

    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      tag_key,
      config.tag_anchor_pose,
      poseNoise(
        config.tag_anchor_translation_sigma_m,
        config.tag_anchor_rotation_sigma_rad)));
    ++result.prior_factor_count;
    result.anchor_strategy = "tag_prior:" + std::to_string(config.tag_anchor_id);
  } else {
    const auto &first_keyframe = session.keyframes.front();
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      xKey(first_keyframe.kf_id),
      first_keyframe.optimized_pose_wb,
      poseNoise(config.anchor_translation_sigma_m, config.anchor_rotation_sigma_rad)));
    ++result.prior_factor_count;
    result.anchor_strategy = "first_keyframe_fallback:" + std::to_string(first_keyframe.kf_id);
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
