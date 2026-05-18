#include <visual_inertial_localization/localization.hpp>
#include <visual_inertial_localization/localization_controller.hpp>
#include <visual_inertial_localization/localization_engine.hpp>

#include <memory>

namespace visual_inertial_localization
{

LocalizationModule::LocalizationModule(LocalizationConfig config)
    : engine_(std::make_unique<LocalizationEngine>(std::move(config))),
      controller_(std::make_unique<LocalizationController>(*engine_))
{
}

LocalizationModule::~LocalizationModule() = default;

LocalizationModule::LocalizationModule(LocalizationModule &&) noexcept = default;

LocalizationModule &LocalizationModule::operator=(LocalizationModule &&) noexcept = default;

LocalizationLoadReport LocalizationModule::loadTagMap()
{
    return engine_->loadTagMap();
}

TagIngestReport LocalizationModule::ingestDetections(
    const apriltag_msgs::msg::AprilTagDetectionArray &msg,
    const std::string &body_frame_id,
    const std::string &odom_frame_id,
    tf2_ros::Buffer &tf_buffer) const
{
    return engine_->ingestDetections(msg, body_frame_id, odom_frame_id, tf_buffer);
}

LocalizationDecision LocalizationModule::processKeyframe(
    int64_t stamp_ns,
    const Eigen::Isometry3d &T_OB) const
{
    return controller_->processKeyframe(stamp_ns, T_OB);
}

void LocalizationModule::updateMapOdomEstimate(const Eigen::Isometry3d &T_MO) const noexcept
{
    controller_->updateMapOdomEstimate(T_MO);
}

LocalizationState LocalizationModule::state() const noexcept
{
    return controller_->state();
}

std::optional<StableCorrectionEstimate> LocalizationModule::estimateStableCorrection(
    const rclcpp::Time &stamp) const
{
    return engine_->estimateStableCorrection(stamp);
}

const LocalizationConfig &LocalizationModule::config() const noexcept
{
    return engine_->config();
}

} // namespace visual_inertial_localization
