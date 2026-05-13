#include "visual_inertial/transport/common_transport.hpp"

namespace visual_inertial::transport
{

builtin_interfaces::msg::Time toBuiltinTime(double tsec)
{
    builtin_interfaces::msg::Time stamp{};
    stamp.sec = static_cast<int32_t>(tsec);
    stamp.nanosec = static_cast<uint32_t>((tsec - stamp.sec) * 1e9);
    return stamp;
}

Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose &pose)
{
    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    q.normalize();

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = q.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    return transform;
}

geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d &transform)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();

    Eigen::Quaterniond q(transform.linear());
    q.normalize();
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    return pose;
}

Eigen::Isometry3d transformMsgToIso(const geometry_msgs::msg::Transform &transform)
{
    Eigen::Quaterniond q(
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z);
    q.normalize();

    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear() = q.toRotationMatrix();
    out.translation() = Eigen::Vector3d(
        transform.translation.x,
        transform.translation.y,
        transform.translation.z);
    return out;
}

geometry_msgs::msg::TransformStamped isoToTf(
    const Eigen::Isometry3d &transform_parent_child,
    const rclcpp::Time &stamp,
    const std::string &parent_frame,
    const std::string &child_frame)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_frame;

    tf.transform.translation.x = transform_parent_child.translation().x();
    tf.transform.translation.y = transform_parent_child.translation().y();
    tf.transform.translation.z = transform_parent_child.translation().z();

    Eigen::Quaterniond q(transform_parent_child.linear());
    q.normalize();
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    return tf;
}

} // namespace visual_inertial::transport
