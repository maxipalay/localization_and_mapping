// body_path_node.cpp
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <deque>
#include <string>
#include <chrono>

class BodyPathNode final : public rclcpp::Node {
public:
  BodyPathNode()
  : rclcpp::Node("body_path_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
    body_frame_  = declare_parameter<std::string>("body_frame",  "body");
    topic_       = declare_parameter<std::string>("topic",       "body_path");
    publish_hz_  = declare_parameter<double>("publish_hz",       10.0);
    history_sec_ = declare_parameter<double>("history_sec",      20.0);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_hz_),
        std::bind(&BodyPathNode::tick_, this));

    RCLCPP_INFO(get_logger(),
                "Publishing Path topic=%s from TF %s->%s @ %.1f Hz, history=%.1fs",
                topic_.c_str(), fixed_frame_.c_str(), body_frame_.c_str(),
                publish_hz_, history_sec_);
  }

private:
  void tick_()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(fixed_frame_, body_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = tf.header.stamp;
    ps.header.frame_id = fixed_frame_;
    ps.pose.position.x = tf.transform.translation.x;
    ps.pose.position.y = tf.transform.translation.y;
    ps.pose.position.z = tf.transform.translation.z;
    ps.pose.orientation = tf.transform.rotation;

    poses_.push_back(ps);

    const rclcpp::Time now = this->now();
    while (!poses_.empty() && (now - poses_.front().header.stamp).seconds() > history_sec_) {
      poses_.pop_front();
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now;
    path.header.frame_id = fixed_frame_;
    path.poses.assign(poses_.begin(), poses_.end());
    path_pub_->publish(path);
  }

private:
  std::string fixed_frame_;
  std::string body_frame_;
  std::string topic_;
  double publish_hz_{10.0};
  double history_sec_{20.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<geometry_msgs::msg::PoseStamped> poses_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyPathNode>());
  rclcpp::shutdown();
  return 0;
}