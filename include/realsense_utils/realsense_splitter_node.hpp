#ifndef REALSENSE_UTILS__REALSENSE_SPLITTER_NODE_HPP_
#define REALSENSE_UTILS__REALSENSE_SPLITTER_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/metadata.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace realsense_utils
{

class RealsenseSplitterNode : public rclcpp::Node
{
public:
  explicit RealsenseSplitterNode(const rclcpp::NodeOptions & options);
  ~RealsenseSplitterNode() override = default;

  enum class EmitterMode : int
  {
    kOn = 1,
    kOff = 0,
    kUnknown = 2
  };

  void image1Callback(
    sensor_msgs::msg::Image::ConstSharedPtr image,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);
  void image2Callback(
    sensor_msgs::msg::Image::ConstSharedPtr image,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);
  void depthCallback(
    sensor_msgs::msg::Image::ConstSharedPtr image,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);
  void pointcloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);

private:
  using image_time_policy_t = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image,
    realsense2_camera_msgs::msg::Metadata>;
  using pointcloud_time_policy_t = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::PointCloud2,
    realsense2_camera_msgs::msg::Metadata>;

  int getEmitterModeFromMetadataMsg(
    const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata);

  template<typename MessageType>
  void republishIfEmitterMode(
    const typename MessageType::ConstSharedPtr & message,
    const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata,
    EmitterMode emitter_mode,
    typename rclcpp::Publisher<MessageType>::SharedPtr & publisher);

  std::shared_ptr<message_filters::Synchronizer<image_time_policy_t>> timesync_infra_1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> infra_1_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata> infra_1_metadata_sub_;

  std::shared_ptr<message_filters::Synchronizer<image_time_policy_t>> timesync_infra_2_;
  message_filters::Subscriber<sensor_msgs::msg::Image> infra_2_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata> infra_2_metadata_sub_;

  std::shared_ptr<message_filters::Synchronizer<image_time_policy_t>> timesync_depth_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata> depth_metadata_sub_;

  std::shared_ptr<message_filters::Synchronizer<pointcloud_time_policy_t>> timesync_pointcloud_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata> pointcloud_metadata_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra_1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra_2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

}  // namespace realsense_utils

#endif  // REALSENSE_UTILS__REALSENSE_SPLITTER_NODE_HPP_
