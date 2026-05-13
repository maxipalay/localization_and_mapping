#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>

#include <Eigen/Geometry>

#include <string>

namespace visual_inertial::transport
{

builtin_interfaces::msg::Time toBuiltinTime(double tsec);

Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose &pose);

geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d &transform);

Eigen::Isometry3d transformMsgToIso(const geometry_msgs::msg::Transform &transform);

geometry_msgs::msg::TransformStamped isoToTf(
    const Eigen::Isometry3d &transform_parent_child,
    const rclcpp::Time &stamp,
    const std::string &parent_frame,
    const std::string &child_frame);

} // namespace visual_inertial::transport
