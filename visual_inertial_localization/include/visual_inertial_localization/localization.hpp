#pragma once

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
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

struct BufferedTagObservation
{
    int64_t stamp_ns{0};
    int tag_id{0};
    std::string family;
    Eigen::Isometry3d T_BT = Eigen::Isometry3d::Identity();
    double decision_margin{0.0};
    double goodness{0.0};
    int hamming{0};
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

class LocalizationModule
{
public:
    explicit LocalizationModule(LocalizationConfig config);

    LocalizationLoadReport loadTagMap();
    TagIngestReport ingestDetections(
        const apriltag_msgs::msg::AprilTagDetectionArray &msg,
        const std::string &body_frame_id,
        tf2_ros::Buffer &tf_buffer) const;
    std::vector<BufferedTagObservation> recentObservationsForStamp(const rclcpp::Time &stamp) const;
    std::optional<BootstrapEstimate> estimateBootstrap(
        const rclcpp::Time &stamp,
        const Eigen::Isometry3d &T_OB) const;
    std::optional<PosePriorEstimate> estimatePosePrior(const rclcpp::Time &stamp) const;
    size_t bufferedObservationCount() const noexcept;

    const LocalizationConfig &config() const noexcept;
    const std::unordered_map<int, Eigen::Isometry3d> &mappedTags() const noexcept;

private:
    std::string tagFrameName_(const apriltag_msgs::msg::AprilTagDetection &detection) const;
    bool isObliqueTagObservation_(const Eigen::Isometry3d &T_BT) const;
    void pruneBufferedTagObservationsLocked_(int64_t newest_stamp_ns) const;

    LocalizationConfig config_;
    std::unordered_map<int, Eigen::Isometry3d> mapped_tags_;
    mutable std::mutex tag_obs_mtx_;
    mutable std::deque<BufferedTagObservation> tag_observation_buffer_;
};

} // namespace visual_inertial_localization
