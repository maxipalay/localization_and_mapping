#ifndef REALSENSE_UTILS__INFRA_GAIN_CORRECTION_NODE_HPP_
#define REALSENSE_UTILS__INFRA_GAIN_CORRECTION_NODE_HPP_

#include <opencv2/core.hpp>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace realsense_utils
{

class InfraGainCorrectionNode : public rclcpp::Node
{
public:
  explicit InfraGainCorrectionNode(const rclcpp::NodeOptions & options);
  ~InfraGainCorrectionNode() override = default;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image);
  void loadGainMap(const std::string & path);
  sensor_msgs::msg::Image correctImage(const sensor_msgs::msg::Image::ConstSharedPtr & image);
  const cv::Mat & gainMapForSize(const cv::Size & image_size);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  cv::Mat gain_map_;
  cv::Mat resized_gain_map_;
  cv::Size resized_gain_map_size_;
  bool resize_gain_map_to_image_{true};
};

}  // namespace realsense_utils

#endif  // REALSENSE_UTILS__INFRA_GAIN_CORRECTION_NODE_HPP_
