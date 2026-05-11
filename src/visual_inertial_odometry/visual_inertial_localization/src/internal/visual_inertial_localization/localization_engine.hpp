#pragma once

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <visual_inertial_localization/localization_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>

#include <atomic>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace visual_inertial_localization
{

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

class LocalizationEngine
{
public:
    explicit LocalizationEngine(LocalizationConfig config);

    LocalizationLoadReport loadTagMap();
    TagIngestReport ingestDetections(
        const apriltag_msgs::msg::AprilTagDetectionArray &msg,
        const std::string &body_frame_id,
        const std::string &odom_frame_id,
        tf2_ros::Buffer &tf_buffer) const;
    std::vector<BufferedTagObservation> recentObservationsForStamp(const rclcpp::Time &stamp) const;
    std::optional<BootstrapEstimate> estimateBootstrap(
        const rclcpp::Time &stamp,
        const Eigen::Isometry3d &T_OB) const;
    std::optional<PosePriorEstimate> estimatePosePrior(const rclcpp::Time &stamp) const;
    std::vector<PosePriorEstimate> estimatePosePriors(const rclcpp::Time &stamp) const;
    std::optional<StableCorrectionEstimate> estimateStableCorrection(const rclcpp::Time &stamp) const;
    std::optional<StableCorrectionEstimate> estimateRelocalizationCorrection(const rclcpp::Time &stamp) const;
    size_t bufferedObservationCount() const noexcept;

    const LocalizationConfig &config() const noexcept;
    const std::unordered_map<int, Eigen::Isometry3d> &mappedTags() const noexcept;

private:
    struct PoseHypothesis
    {
        int tag_id{0};
        int64_t stamp_ns{0};
        Eigen::Isometry3d T_MB = Eigen::Isometry3d::Identity();
        double score{0.0};
        double decision_margin{0.0};
    };

    struct TemporalCorrectionHypothesis
    {
        int64_t stamp_ns{0};
        Eigen::Isometry3d T_MO = Eigen::Isometry3d::Identity();
        double score{0.0};
        size_t support_count{0};
    };

    std::string tagFrameName_(const apriltag_msgs::msg::AprilTagDetection &detection) const;
    bool isObliqueTagObservation_(const Eigen::Isometry3d &T_BT) const;
    void pruneBufferedTagObservationsLocked_(int64_t newest_stamp_ns) const;
    std::vector<PoseHypothesis> buildPoseHypotheses_(
        const std::vector<BufferedTagObservation> &observations) const;
    std::vector<size_t> dominantHypothesisCluster_(
        const std::vector<PoseHypothesis> &hypotheses) const;
    size_t bestHypothesisIndex_(
        const std::vector<PoseHypothesis> &hypotheses,
        const std::vector<size_t> &cluster_indices) const;
    void pruneTemporalCorrectionHypothesesLocked_(int64_t newest_stamp_ns) const;
    std::optional<StableCorrectionEstimate> computeCorrectionEstimateLocked_(
        size_t min_frame_support) const;
    std::optional<StableCorrectionEstimate> updateStableCorrection_(
        int64_t stamp_ns,
        const Eigen::Isometry3d &T_MO,
        double score,
        size_t support_count) const;

    LocalizationConfig config_;
    std::unordered_map<int, Eigen::Isometry3d> mapped_tags_;
    mutable std::mutex tag_obs_mtx_;
    mutable std::deque<BufferedTagObservation> tag_observation_buffer_;
    mutable std::mutex stable_correction_mtx_;
    mutable std::deque<TemporalCorrectionHypothesis> stable_correction_buffer_;
    mutable std::atomic<int64_t> last_tag_message_stamp_ns_{0};
};

} // namespace visual_inertial_localization
