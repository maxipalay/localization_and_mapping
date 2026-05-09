#pragma once

#include <visual_inertial_localization/localization_controller.hpp>
#include <visual_inertial_localization/localization_engine.hpp>

namespace visual_inertial_localization
{

class LocalizationModule
{
public:
    explicit LocalizationModule(LocalizationConfig config);

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
    LocalizationEngine engine_;
    LocalizationController controller_;
};

} // namespace visual_inertial_localization
