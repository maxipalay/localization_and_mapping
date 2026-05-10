#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <unordered_map>

namespace visual_inertial_localization
{

struct LocalizationConfig
{
    std::string tag_map_path;
    std::string tag_topic{"detections"};
    double tag_tf_lookup_timeout_ms{50.0};
    double tag_max_age_s{0.5};
    double tag_buffer_age_s{2.0};
    int max_tag_hamming{0};
    double min_tag_decision_margin{40.0};
    double max_tag_range_m{3.0};
    double max_tag_oblique_angle_deg{40.0};
    double pose_prior_rot_sigma_rad{0.35};
    double pose_prior_trans_sigma_m{0.10};
    double pose_prior_huber_k{1.0};
    double cluster_translation_m{0.75};
    double cluster_rotation_deg{20.0};
    double stable_hypothesis_age_s{1.0};
    size_t stable_min_frames{3};
    size_t relocalization_min_history_frames{5};
    double stable_translation_m{0.25};
    double stable_rotation_deg{8.0};
    double relocalize_translation_m{0.25};
    double relocalize_rotation_deg{8.0};
    double tracking_deadband_translation_m{0.05};
    double tracking_deadband_rotation_deg{2.0};
    std::unordered_map<int, std::string> tag_frame_overrides;
};

struct LocalizationLoadReport
{
    bool ok{false};
    std::string message;
    size_t mapped_tag_count{0};
    size_t frame_override_count{0};
};

struct TagIngestReport
{
    size_t total_detections{0};
    size_t accepted{0};
    size_t skipped_unmapped{0};
    size_t skipped_hamming{0};
    size_t skipped_margin{0};
    size_t skipped_tf_lookup{0};
    size_t skipped_range{0};
    size_t skipped_oblique{0};
    size_t buffered{0};
};

struct BootstrapEstimate
{
    Eigen::Isometry3d T_MO = Eigen::Isometry3d::Identity();
    size_t support_count{0};
    double score{0.0};
};

struct PosePriorEstimate
{
    Eigen::Isometry3d T_MB = Eigen::Isometry3d::Identity();
    size_t support_count{0};
    double score{0.0};
};

struct StableCorrectionEstimate
{
    Eigen::Isometry3d T_MO = Eigen::Isometry3d::Identity();
    size_t frame_support{0};
    size_t support_count{0};
    double score{0.0};
};

} // namespace visual_inertial_localization
