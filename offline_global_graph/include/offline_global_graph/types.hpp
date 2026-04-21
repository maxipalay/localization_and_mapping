#pragma once

#include <gtsam/geometry/Pose3.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
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
    double min_track_retention{1.0};
    double mean_track_retention{1.0};
    double mean_pnp_inlier_ratio{1.0};
    double max_pnp_reproj_rmse_px{0.0};
    double min_track_coverage{1.0};
    double mean_track_coverage{1.0};
  };

  uint64_t kf_id{0};
  int64_t stamp_ns{0};
  std::filesystem::path rgb_path;
  std::filesystem::path depth_path;
  std::filesystem::path tags_path;
  std::filesystem::path keyframe_meta_path;
  gtsam::Pose3 frontend_pose_wc;
  gtsam::Pose3 initial_pose_wb;
  gtsam::Pose3 optimized_pose_wb;
  std::optional<gtsam::Pose3> between_pose_prev_curr_body;
  IntervalHealth interval_health;
  bool has_pose_wb_covariance{false};
  std::array<double, 36> pose_wb_covariance{};
  struct TrackObservation
  {
    uint32_t track_id{0};
    float u_l{0.0f};
    float v_l{0.0f};
    float u_r{0.0f};
    float v_r{0.0f};
    bool has_right{false};
  };
  std::vector<TrackObservation> track_observations;
};

struct CameraIntrinsics
{
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string frame_id;
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

struct SessionData
{
  std::filesystem::path session_dir;
  std::string session_name;
  std::string body_frame_id{"body"};
  CameraIntrinsics camera;
  std::vector<KeyframeRecord> keyframes;
  std::vector<TagObservation> tag_observations;
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
  double anchor_translation_sigma_m{0.30};
  double anchor_rotation_sigma_rad{0.50};
  double robust_huber_k{1.345};
  double tag_observation_huber_k{1.345};
  bool use_interval_health_for_between{true};
  double between_health_min_pose_valid_fraction{0.6};
  double between_health_min_track_retention{0.6};
  double between_health_min_pnp_inlier_ratio{0.25};
  double between_health_min_track_coverage{0.2};
  double between_health_max_pnp_reproj_rmse_px{3.0};
  double between_health_max_sigma_scale{5.0};
  double between_health_skip_quality{0.05};
  bool use_optimizer_pose_priors{true};
  double optimizer_pose_prior_covariance_scale{10.0};
  double optimizer_pose_prior_min_translation_sigma_m{0.10};
  double optimizer_pose_prior_min_rotation_sigma_rad{0.10};
  double optimizer_pose_prior_huber_k{1.345};
  bool use_visual_factors{false};
  gtsam::Pose3 body_T_camera;
  double visual_sigma_px{1.0};
  double visual_huber_k{1.345};
  double stereo_baseline_m{0.0};
  double depth_scale{0.001};
  double min_depth_m{0.3};
  double max_depth_m{6.0};
  int min_track_observations{2};
};

struct OptimizationResult
{
  std::vector<std::pair<uint64_t, gtsam::Pose3>> optimized_keyframes;
  std::vector<std::pair<int, gtsam::Pose3>> optimized_tags;
  double initial_error{0.0};
  double final_error{0.0};
  size_t between_factor_count{0};
  size_t between_factor_inflated_count{0};
  size_t between_factor_skipped_count{0};
  size_t tag_observation_factor_count{0};
  size_t prior_factor_count{0};
  size_t optimizer_pose_prior_count{0};
  size_t visual_factor_count{0};
  size_t landmark_count{0};
  std::string anchor_strategy;
};

}  // namespace offline_global_graph
