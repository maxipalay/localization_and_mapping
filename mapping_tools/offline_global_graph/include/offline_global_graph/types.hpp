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
  struct IntervalHealth
  {
    uint32_t num_frames{0};
    uint32_t num_pose_valid_frames{0};
    uint32_t num_degraded_frames{0};
    uint32_t num_lost_frames{0};
    int32_t min_tracks{0};
    double mean_tracks{0.0};
    double min_track_retention{0.0};
    double mean_track_retention{0.0};
    double mean_pnp_inlier_ratio{0.0};
    double max_pnp_reproj_rmse_px{0.0};
    double min_track_coverage{0.0};
    double mean_track_coverage{0.0};
  };

  uint64_t kf_id{0};
  int64_t stamp_ns{0};
  std::filesystem::path tags_path;
  std::filesystem::path keyframe_meta_path;
  gtsam::Pose3 optimized_pose_wb;
  bool has_vo_between_measurement{false};
  gtsam::Pose3 vo_between_prev_curr_body;
  IntervalHealth interval_health;
};

struct TagObservation
{
  uint64_t kf_id{0};
  int tag_id{0};
  std::string family;
  int hamming{0};
  double decision_margin{0.0};
  std::string parent_frame_id;
  std::string child_frame_id;
  std::string detection_frame_id;
  int64_t lookup_stamp_ns{0};
  gtsam::Pose3 body_T_tag;
  std::filesystem::path source_path;
};

struct SessionData
{
  std::filesystem::path session_dir;
  std::string session_name;
  std::string body_frame_id{"body"};
  std::vector<KeyframeRecord> keyframes;
  std::vector<TagObservation> tag_observations;
  size_t skipped_missing_tf_tag_observations{0};
  size_t skipped_unavailable_tag_observations{0};
  size_t skipped_non_body_tag_observations{0};
};

struct OptimizerConfig
{
  double between_translation_sigma_m{0.05};  // Translation sigma for consecutive-keyframe between factors.
  double between_rotation_sigma_rad{0.1};  // Rotation sigma for consecutive-keyframe between factors.
  bool use_interval_health_for_between{true};  // Scale or skip consecutive factors using logged interval health.
  double between_health_min_pose_valid_fraction{0.6};
  double between_health_min_track_retention{0.6};
  double between_health_min_pnp_inlier_ratio{0.25};
  double between_health_min_track_coverage{0.2};
  double between_health_max_pnp_reproj_rmse_px{3.0};
  double between_health_max_sigma_scale{5.0};
  double between_health_skip_quality{0.4};
  double tag_translation_sigma_m{0.10};  // Translation sigma for body-to-tag observation factors.
  double tag_rotation_sigma_rad{1.57};  // Rotation sigma for body-to-tag observation factors.
  bool use_tag_anchor_prior{false};  // Add a prior on a chosen tag instead of anchoring the first keyframe.
  int tag_anchor_id{0};  // Tag id to anchor when use_tag_anchor_prior is enabled.
  gtsam::Pose3 tag_anchor_pose;  // World pose used by the tag anchor prior.
  double tag_anchor_translation_sigma_m{0.05};  // Translation sigma of the anchor-tag prior.
  double tag_anchor_rotation_sigma_rad{0.3};  // Rotation sigma of the anchor-tag prior.
  bool reject_distant_tag_observations{true};  // Reject tag observations beyond the configured range limit.
  double max_tag_observation_distance_m{3.0};  // Maximum allowed body-to-tag range before rejection.
  bool reject_oblique_tag_observations{true};  // Reject tag observations seen at too oblique an angle.
  double max_tag_oblique_angle_deg{40.0};  // Maximum angle between tag normal and viewing direction.
  bool reject_tag_observations_with_hamming{true};  // Reject detections whose decoded tag required too many bit corrections.
  int max_tag_hamming{0};  // Maximum allowed hamming distance before rejection.
  bool reject_low_margin_tag_observations{true};  // Reject detections with weak AprilTag decision margin.
  double min_tag_decision_margin{40.0};  // Minimum allowed decision margin before rejection.
  double anchor_translation_sigma_m{0.05};  // Translation sigma of the fallback first-keyframe anchor.
  double anchor_rotation_sigma_rad{0.3};  // Rotation sigma of the fallback first-keyframe anchor.
  double tag_observation_huber_k{1.0};  // Huber loss threshold for tag observation factors.
};

struct OptimizationResult
{
  std::vector<std::pair<uint64_t, gtsam::Pose3>> optimized_keyframes;
  std::vector<std::pair<int, gtsam::Pose3>> optimized_tags;
  double initial_error{0.0};
  double final_error{0.0};
  size_t between_factor_count{0};
  size_t between_factor_low_quality_count{0};
  size_t tag_observation_factor_count{0};
  size_t tag_observation_skipped_distance_count{0};
  size_t tag_observation_skipped_oblique_count{0};
  size_t tag_observation_skipped_hamming_count{0};
  size_t tag_observation_skipped_low_margin_count{0};
  size_t prior_factor_count{0};
  double min_between_interval_quality{1.0};
  std::string anchor_strategy;
};

}  // namespace offline_global_graph
