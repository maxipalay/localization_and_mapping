#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

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
    size_t bootstrap_min_inliers{2};
    double stable_hypothesis_age_s{1.0};
    size_t stable_min_frames{3};
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

class LocalizationModule
{
public:
    explicit LocalizationModule(LocalizationConfig config);

    LocalizationLoadReport loadTagMap();

    const LocalizationConfig &config() const noexcept;
    const std::unordered_map<int, Eigen::Isometry3d> &mappedTags() const noexcept;

private:
    LocalizationConfig config_;
    std::unordered_map<int, Eigen::Isometry3d> mapped_tags_;
};

} // namespace visual_inertial_localization
