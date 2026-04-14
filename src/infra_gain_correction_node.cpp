#include "realsense_utils/infra_gain_correction_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

#include <opencv2/imgproc.hpp>

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

cv::Mat loadNpyFloat32Matrix(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("failed to open file");
  }

  std::array<char, 6> magic{};
  file.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!file || std::string(magic.data(), magic.size()) != "\x93NUMPY") {
    throw std::runtime_error("invalid npy header");
  }

  std::array<std::uint8_t, 2> version{};
  file.read(reinterpret_cast<char *>(version.data()), static_cast<std::streamsize>(version.size()));
  if (!file) {
    throw std::runtime_error("failed to read npy version");
  }

  std::uint32_t header_length = 0;
  if (version[0] == 1) {
    std::uint16_t header_length_v1 = 0;
    file.read(reinterpret_cast<char *>(&header_length_v1), sizeof(header_length_v1));
    header_length = header_length_v1;
  } else if (version[0] == 2 || version[0] == 3) {
    file.read(reinterpret_cast<char *>(&header_length), sizeof(header_length));
  } else {
    throw std::runtime_error("unsupported npy version");
  }
  if (!file) {
    throw std::runtime_error("failed to read npy header length");
  }

  std::string header(header_length, '\0');
  file.read(header.data(), static_cast<std::streamsize>(header.size()));
  if (!file) {
    throw std::runtime_error("failed to read npy header");
  }

  const std::regex descr_regex(R"('descr'\s*:\s*'([^']+)')");
  const std::regex fortran_regex(R"('fortran_order'\s*:\s*(True|False))");
  const std::regex shape_regex(R"('shape'\s*:\s*\(\s*(\d+)\s*,\s*(\d+)\s*,?\s*\))");
  std::smatch match;

  if (!std::regex_search(header, match, descr_regex)) {
    throw std::runtime_error("npy header missing descr");
  }
  const std::string descr = match[1].str();
  if (descr != "<f4" && descr != "|f4") {
    throw std::runtime_error("expected float32 npy matrix");
  }

  if (!std::regex_search(header, match, fortran_regex)) {
    throw std::runtime_error("npy header missing fortran_order");
  }
  if (match[1].str() != "False") {
    throw std::runtime_error("fortran-order npy arrays are not supported");
  }

  if (!std::regex_search(header, match, shape_regex)) {
    throw std::runtime_error("expected 2D npy matrix");
  }
  const int rows = std::stoi(match[1].str());
  const int cols = std::stoi(match[2].str());
  if (rows <= 0 || cols <= 0) {
    throw std::runtime_error("invalid npy matrix shape");
  }

  cv::Mat matrix(rows, cols, CV_32F);
  const auto bytes_to_read =
    static_cast<std::streamsize>(rows) * static_cast<std::streamsize>(cols) *
    static_cast<std::streamsize>(sizeof(float));
  file.read(reinterpret_cast<char *>(matrix.data), bytes_to_read);
  if (!file) {
    throw std::runtime_error("failed to read npy matrix data");
  }
  return matrix;
}

}  // namespace

InfraGainCorrectionNode::InfraGainCorrectionNode(const rclcpp::NodeOptions & options)
: Node("infra_gain_correction_node", options)
{
  constexpr size_t kQueueSize = 10;
  const auto input_qos_name = declare_parameter<std::string>("input_qos", "SYSTEM_DEFAULT");
  const auto output_qos_name = declare_parameter<std::string>("output_qos", "SYSTEM_DEFAULT");
  const auto gain_map_path = declare_parameter<std::string>("gain_map_path", "");
  resize_gain_map_to_image_ = declare_parameter<bool>("resize_gain_map_to_image", true);

  const rclcpp::QoS input_qos = qosFromString(input_qos_name, kQueueSize);
  const rclcpp::QoS output_qos = qosFromString(output_qos_name, kQueueSize);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "input/image",
    input_qos,
    std::bind(&InfraGainCorrectionNode::imageCallback, this, std::placeholders::_1));
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/image", output_qos);

  if (!gain_map_path.empty()) {
    loadGainMap(gain_map_path);
  } else {
    RCLCPP_WARN(get_logger(), "No gain_map_path configured. Node will pass images through.");
  }
}

void InfraGainCorrectionNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image)
{
  image_pub_->publish(correctImage(image));
}

void InfraGainCorrectionNode::loadGainMap(const std::string & path)
{
  try {
    gain_map_ = loadNpyFloat32Matrix(path);
    resized_gain_map_ = gain_map_;
    resized_gain_map_size_ = gain_map_.size();
    const auto [min_it, max_it] = std::minmax_element(
      gain_map_.begin<float>(), gain_map_.end<float>());
    RCLCPP_INFO(
      get_logger(),
      "Loaded gain map from %s (%dx%d, min=%.3f, max=%.3f)",
      path.c_str(), gain_map_.cols, gain_map_.rows,
      static_cast<double>(*min_it), static_cast<double>(*max_it));
  } catch (const std::exception & ex) {
    gain_map_.release();
    resized_gain_map_.release();
    resized_gain_map_size_ = {};
    RCLCPP_ERROR(get_logger(), "Failed to load gain map %s: %s", path.c_str(), ex.what());
  }
}

const cv::Mat & InfraGainCorrectionNode::gainMapForSize(const cv::Size & image_size)
{
  if (gain_map_.empty()) {
    throw std::runtime_error("gain map is not loaded");
  }
  if (gain_map_.size() == image_size) {
    return gain_map_;
  }
  if (!resize_gain_map_to_image_) {
    throw std::runtime_error("gain map size does not match the input image");
  }
  if (resized_gain_map_.empty() || resized_gain_map_size_ != image_size) {
    cv::resize(gain_map_, resized_gain_map_, image_size, 0.0, 0.0, cv::INTER_LINEAR);
    resized_gain_map_size_ = image_size;
    RCLCPP_INFO(
      get_logger(),
      "Resized gain map from %dx%d to %dx%d",
      gain_map_.cols, gain_map_.rows, image_size.width, image_size.height);
  }
  return resized_gain_map_;
}

sensor_msgs::msg::Image InfraGainCorrectionNode::correctImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  sensor_msgs::msg::Image corrected_msg = *image;
  if (gain_map_.empty()) {
    return corrected_msg;
  }

  try {
    const auto cv_image = cv_bridge::toCvShare(image, image->encoding);
    const auto & gain_map = gainMapForSize(cv_image->image.size());
    cv::Mat src_f32;
    cv::Mat corrected_f32;
    cv::Mat corrected;

    if (
      image->encoding == sensor_msgs::image_encodings::MONO8 ||
      image->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
    {
      cv_image->image.convertTo(src_f32, CV_32F);
      cv::multiply(src_f32, gain_map, corrected_f32);
      corrected_f32.convertTo(corrected, CV_8U);
    } else if (
      image->encoding == sensor_msgs::image_encodings::MONO16 ||
      image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      cv_image->image.convertTo(src_f32, CV_32F);
      cv::multiply(src_f32, gain_map, corrected_f32);
      corrected_f32.convertTo(corrected, CV_16U);
    } else {
      constexpr int kPublishPeriodMs = 3000;
      auto & clock = *get_clock();
      RCLCPP_WARN_THROTTLE(
        get_logger(), clock, kPublishPeriodMs,
        "Gain correction only supports mono8/mono16 images. Passing through encoding %s.",
        image->encoding.c_str());
      return corrected_msg;
    }

    return *cv_bridge::CvImage(image->header, image->encoding, corrected).toImageMsg();
  } catch (const std::exception & ex) {
    constexpr int kPublishPeriodMs = 3000;
    auto & clock = *get_clock();
    RCLCPP_WARN_THROTTLE(
      get_logger(), clock, kPublishPeriodMs,
      "Failed to apply gain correction: %s. Passing through raw image.", ex.what());
    return corrected_msg;
  }
}

}  // namespace realsense_utils

RCLCPP_COMPONENTS_REGISTER_NODE(realsense_utils::InfraGainCorrectionNode)
