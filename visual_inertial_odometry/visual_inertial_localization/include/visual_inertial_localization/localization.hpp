#pragma once

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <visual_inertial_localization/localization_data.hpp>
#include <visual_inertial_localization/localization_types.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>

namespace visual_inertial_localization
{

class LocalizationController;
class LocalizationEngine;

class LocalizationModule
{
public:
    explicit LocalizationModule(LocalizationConfig config);
    ~LocalizationModule();

    LocalizationModule(const LocalizationModule &) = delete;
    LocalizationModule &operator=(const LocalizationModule &) = delete;
    LocalizationModule(LocalizationModule &&) noexcept;
    LocalizationModule &operator=(LocalizationModule &&) noexcept;

    LocalizationLoadReport loadTagMap();
    TagIngestReport ingestDetections(
        const apriltag_msgs::msg::AprilTagDetectionArray &msg,
        const std::string &body_frame_id,
        const std::string &odom_frame_id,
        tf2_ros::Buffer &tf_buffer) const;
    LocalizationDecision processKeyframe(
        int64_t stamp_ns,
        const Eigen::Isometry3d &T_OB) const;
    void updateMapOdomEstimate(const Eigen::Isometry3d &T_MO) const noexcept;
    LocalizationState state() const noexcept;
    std::optional<StableCorrectionEstimate> estimateStableCorrection(
        const rclcpp::Time &stamp) const;

    const LocalizationConfig &config() const noexcept;

private:
    std::unique_ptr<LocalizationEngine> engine_;
    std::unique_ptr<LocalizationController> controller_;
};

} // namespace visual_inertial_localization
