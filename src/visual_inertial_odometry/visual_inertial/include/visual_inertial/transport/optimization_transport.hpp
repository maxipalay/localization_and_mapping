#pragma once

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visual_inertial/msg/imu_bias.hpp>
#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/localization_feedback.hpp>
#include <visual_inertial/msg/optimization_result.hpp>
#include <visual_inertial_common/types.hpp>
#include <visual_inertial_optimization/optimization.hpp>

#include <string>
#include <vector>

namespace visual_inertial::transport
{

KeyframeEvent toKeyframeEvent(const visual_inertial::msg::Keyframe &msg);

visual_inertial::msg::ImuBias makeImuBiasMsg(
    const visual_inertial::msg::Keyframe &msg,
    const OptimizationResult &result);

visual_inertial::msg::OptimizationResult makeOptimizationResultMsg(
    const visual_inertial::msg::Keyframe &msg,
    const OptimizationResult &result,
    const std::string &map_frame_id);

visual_inertial::msg::LocalizationFeedback makeLocalizationFeedbackMsg(
    const visual_inertial::msg::Keyframe &msg,
    const OptimizationResult &result);

sensor_msgs::msg::PointCloud2 makeLandmarkPointCloudMsg(
    const std::vector<LandmarkEstimate> &landmarks,
    const rclcpp::Time &stamp,
    const std::string &frame_id);

} // namespace visual_inertial::transport
