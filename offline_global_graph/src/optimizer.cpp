#include "offline_global_graph/optimizer.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
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
  for (const auto &keyframe : session.keyframes) {
    initial_values.insert(xKey(keyframe.kf_id), keyframe.initial_pose_wb);
    keyframe_initials.emplace(keyframe.kf_id, keyframe.initial_pose_wb);
  }

  const auto between_noise = poseNoise(
    config.between_translation_sigma_m, config.between_rotation_sigma_rad);
  for (size_t i = 1; i < session.keyframes.size(); ++i) {
    const auto &previous = session.keyframes[i - 1];
    const auto &current = session.keyframes[i];
    const gtsam::Pose3 measurement = current.between_pose_prev_curr_body.has_value() ?
      *current.between_pose_prev_curr_body :
      previous.initial_pose_wb.between(current.initial_pose_wb);
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
        config.robust_huber_k)));
    ++result.tag_observation_factor_count;
  }

  bool anchor_assigned = false;
  for (size_t i = 0; i < session.tag_priors.size(); ++i) {
    const auto &prior = session.tag_priors[i];
    const bool use_anchor_noise = prior.anchor || (!anchor_assigned && i == 0);
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

  return result;
}

}  // namespace offline_global_graph
