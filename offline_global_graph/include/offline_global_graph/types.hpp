#pragma once

#include <gtsam/geometry/Pose3.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace offline_global_graph
{

struct KeyframeRecord
{
  uint64_t kf_id{0};
  int64_t stamp_ns{0};
  std::filesystem::path rgb_path;
  std::filesystem::path depth_path;
  std::filesystem::path tags_path;
  std::filesystem::path keyframe_meta_path;
  gtsam::Pose3 frontend_pose_wc;
  gtsam::Pose3 initial_pose_wb;
};

struct TagObservation
{
  uint64_t kf_id{0};
  int tag_id{0};
  std::string family;
  std::string parent_frame_id;
  std::string child_frame_id;
  std::string detection_frame_id;
  int64_t lookup_stamp_ns{0};
  gtsam::Pose3 body_T_tag;
  std::filesystem::path source_path;
};

struct TagPrior
{
  int tag_id{0};
  gtsam::Pose3 world_T_tag;
  double translation_sigma_m{0.10};
  double rotation_sigma_rad{0.20};
  bool has_custom_sigmas{false};
  bool anchor{false};
  std::string source_name;
};

struct SessionData
{
  std::filesystem::path session_dir;
  std::string session_name;
  std::string body_frame_id{"body"};
  std::vector<KeyframeRecord> keyframes;
  std::vector<TagObservation> tag_observations;
  std::vector<TagPrior> tag_priors;
  size_t skipped_missing_tf_tag_observations{0};
  size_t skipped_unavailable_tag_observations{0};
  size_t skipped_non_body_tag_observations{0};
};

struct OptimizerConfig
{
  double between_translation_sigma_m{0.05};
  double between_rotation_sigma_rad{0.05};
  double tag_translation_sigma_m{0.10};
  double tag_rotation_sigma_rad{0.10};
  double soft_prior_translation_sigma_m{0.10};
  double soft_prior_rotation_sigma_rad{0.20};
  double anchor_translation_sigma_m{0.01};
  double anchor_rotation_sigma_rad{0.01};
  double robust_huber_k{1.345};
};

struct OptimizationResult
{
  std::vector<std::pair<uint64_t, gtsam::Pose3>> optimized_keyframes;
  std::vector<std::pair<int, gtsam::Pose3>> optimized_tags;
  double initial_error{0.0};
  double final_error{0.0};
  size_t between_factor_count{0};
  size_t tag_observation_factor_count{0};
  size_t prior_factor_count{0};
  std::string anchor_strategy;
};

}  // namespace offline_global_graph
