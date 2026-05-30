#include "realsense_utils/realsense_splitter_node.hpp"

#include <functional>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace realsense_utils
{

namespace
{

rclcpp::QoS qosFromString(const std::string & qos_name, size_t queue_size)
{
  if (qos_name == "SENSOR_DATA") {
    return rclcpp::SensorDataQoS().keep_last(queue_size);
  }
  return rclcpp::SystemDefaultsQoS().keep_last(queue_size);
}

}  // namespace

RealsenseSplitterNode::RealsenseSplitterNode(const rclcpp::NodeOptions & options)
: Node("realsense_splitter_node", options)
{
  constexpr size_t kInputQueueSize = 10;
  constexpr size_t kOutputQueueSize = 10;

  const auto input_qos_name = declare_parameter<std::string>("input_qos", "SYSTEM_DEFAULT");
  const auto output_qos_name = declare_parameter<std::string>("output_qos", "SYSTEM_DEFAULT");

  const rclcpp::QoS input_qos = qosFromString(input_qos_name, kInputQueueSize);
  const rclcpp::QoS output_qos = qosFromString(output_qos_name, kOutputQueueSize);
  const rmw_qos_profile_t input_qos_profile = input_qos.get_rmw_qos_profile();

  infra_1_sub_.subscribe(this, "input/infra_1", input_qos_profile);
  infra_1_metadata_sub_.subscribe(this, "input/infra_1_metadata", input_qos_profile);
  infra_2_sub_.subscribe(this, "input/infra_2", input_qos_profile);
  infra_2_metadata_sub_.subscribe(this, "input/infra_2_metadata", input_qos_profile);
  depth_sub_.subscribe(this, "input/depth", input_qos_profile);
  depth_metadata_sub_.subscribe(this, "input/depth_metadata", input_qos_profile);
  pointcloud_sub_.subscribe(this, "input/pointcloud", input_qos_profile);
  pointcloud_metadata_sub_.subscribe(this, "input/pointcloud_metadata", input_qos_profile);

  timesync_infra_1_ = std::make_shared<message_filters::Synchronizer<image_time_policy_t>>(
    image_time_policy_t(kInputQueueSize), infra_1_sub_, infra_1_metadata_sub_);
  timesync_infra_1_->registerCallback(std::bind(
    &RealsenseSplitterNode::image1Callback, this,
    std::placeholders::_1, std::placeholders::_2));

  timesync_infra_2_ = std::make_shared<message_filters::Synchronizer<image_time_policy_t>>(
    image_time_policy_t(kInputQueueSize), infra_2_sub_, infra_2_metadata_sub_);
  timesync_infra_2_->registerCallback(std::bind(
    &RealsenseSplitterNode::image2Callback, this,
    std::placeholders::_1, std::placeholders::_2));

  timesync_depth_ = std::make_shared<message_filters::Synchronizer<image_time_policy_t>>(
    image_time_policy_t(kInputQueueSize), depth_sub_, depth_metadata_sub_);
  timesync_depth_->registerCallback(std::bind(
    &RealsenseSplitterNode::depthCallback, this,
    std::placeholders::_1, std::placeholders::_2));

  timesync_pointcloud_ =
    std::make_shared<message_filters::Synchronizer<pointcloud_time_policy_t>>(
    pointcloud_time_policy_t(kInputQueueSize), pointcloud_sub_, pointcloud_metadata_sub_);
  timesync_pointcloud_->registerCallback(std::bind(
    &RealsenseSplitterNode::pointcloudCallback, this,
    std::placeholders::_1, std::placeholders::_2));

  infra_1_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/infra_1", output_qos);
  infra_2_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/infra_2", output_qos);
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/depth", output_qos);
  pointcloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/output/pointcloud", output_qos);
}

int RealsenseSplitterNode::getEmitterModeFromMetadataMsg(
  const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata)
{
  constexpr char frame_emitter_mode_str[] = "\"frame_emitter_mode\":";
  constexpr size_t field_name_length =
    sizeof(frame_emitter_mode_str) / sizeof(frame_emitter_mode_str[0]);

  const size_t frame_emitter_mode_start_location =
    metadata->json_data.find(frame_emitter_mode_str);
  if (frame_emitter_mode_start_location == metadata->json_data.npos) {
    constexpr int kPublishPeriodMs = 1000;
    auto & clock = *get_clock();
    RCLCPP_WARN_THROTTLE(
      get_logger(), clock, kPublishPeriodMs,
      "Realsense frame metadata did not contain \"frame_emitter_mode\". Splitter will not work.");
    return static_cast<int>(EmitterMode::kUnknown);
  }

  const size_t field_location = frame_emitter_mode_start_location + field_name_length - 1;
  return static_cast<int>(metadata->json_data[field_location]) - static_cast<int>('0');
}

template<typename MessageType>
void RealsenseSplitterNode::republishIfEmitterMode(
  const typename MessageType::ConstSharedPtr & message,
  const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata,
  EmitterMode emitter_mode,
  typename rclcpp::Publisher<MessageType>::SharedPtr & publisher)
{
  if (getEmitterModeFromMetadataMsg(metadata) == static_cast<int>(emitter_mode)) {
    publisher->publish(*message);
  }
}

void RealsenseSplitterNode::image1Callback(
  sensor_msgs::msg::Image::ConstSharedPtr image,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::Image>(image, metadata, EmitterMode::kOff, infra_1_pub_);
}

void RealsenseSplitterNode::image2Callback(
  sensor_msgs::msg::Image::ConstSharedPtr image,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::Image>(image, metadata, EmitterMode::kOff, infra_2_pub_);
}

void RealsenseSplitterNode::depthCallback(
  sensor_msgs::msg::Image::ConstSharedPtr image,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::Image>(image, metadata, EmitterMode::kOn, depth_pub_);
}

void RealsenseSplitterNode::pointcloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::PointCloud2>(
    pointcloud, metadata, EmitterMode::kOn, pointcloud_pub_);
}

}  // namespace realsense_utils

RCLCPP_COMPONENTS_REGISTER_NODE(realsense_utils::RealsenseSplitterNode)
