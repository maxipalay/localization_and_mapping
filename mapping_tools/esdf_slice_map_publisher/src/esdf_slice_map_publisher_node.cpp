#include "map_conversion.hpp"
#include "map_io.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

class EsdfSliceMapPublisherNode final : public rclcpp::Node
{
public:
  EsdfSliceMapPublisherNode()
  : Node("esdf_slice_map_publisher")
  {
    map_yaml_path_ = declare_parameter<std::string>("map_yaml_path", "");
    topic_ = declare_parameter<std::string>("topic", "/esdf_slice_map");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/esdf_slice_pointcloud");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    publish_occupancy_ = declare_parameter<bool>("publish_occupancy", true);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    publish_once_ = declare_parameter<bool>("publish_once", true);
    pointcloud_stride_ = declare_parameter<int>("pointcloud_stride", 1);
    max_pointcloud_distance_m_ = declare_parameter<double>(
      "max_pointcloud_distance_m", std::numeric_limits<double>::infinity());
    const double configured_slice_height_m = declare_parameter<double>("slice_height_m", std::numeric_limits<double>::quiet_NaN());
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

    if (map_yaml_path_.empty()) {
      throw std::runtime_error("map_yaml_path parameter is required");
    }
    if (!publish_occupancy_ && !publish_pointcloud_) {
      throw std::runtime_error("at least one of publish_occupancy or publish_pointcloud must be true");
    }
    if (pointcloud_stride_ <= 0) {
      throw std::runtime_error("pointcloud_stride must be positive");
    }

    const auto spec = esdf_slice_map_publisher::loadMapSpec(map_yaml_path_);
    const auto image = esdf_slice_map_publisher::loadPgm(spec.image_path);
    map_ = esdf_slice_map_publisher::buildOccupancyGrid(image, spec, frame_id_, now());
    if (std::isfinite(configured_slice_height_m)) {
      slice_height_m_ = configured_slice_height_m;
    } else {
      slice_height_m_ = esdf_slice_map_publisher::parseSliceHeightFromPath(spec.yaml_path).value_or(0.0);
    }
    distance_meters_ = esdf_slice_map_publisher::buildDistanceTransformMeters(map_);
    if (publish_pointcloud_) {
      cloud_ = esdf_slice_map_publisher::buildDistancePointCloud(
        map_, distance_meters_, frame_id_, slice_height_m_,
        pointcloud_stride_, max_pointcloud_distance_m_, now());
    }

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    if (publish_occupancy_) {
      pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);
    }
    if (publish_pointcloud_) {
      pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, qos);
    }

    const std::string occupancy_suffix = publish_occupancy_ ?
      ("occupancy on '" + topic_ + "'") :
      "occupancy disabled";
    const std::string pointcloud_suffix = publish_pointcloud_ ?
      ("pointcloud on '" + pointcloud_topic_ + "'") :
      "pointcloud disabled";
    const std::string max_distance_text = std::isfinite(max_pointcloud_distance_m_) ?
      std::to_string(max_pointcloud_distance_m_) :
      "inf";
    RCLCPP_INFO(
      get_logger(),
      "Loaded map '%s' (%ux%u at %.3f m/cell), %s, %s, stride=%d, max_distance=%s, publish_once=%s",
      map_yaml_path_.c_str(),
      map_.info.width,
      map_.info.height,
      map_.info.resolution,
      occupancy_suffix.c_str(),
      pointcloud_suffix.c_str(),
      pointcloud_stride_,
      max_distance_text.c_str(),
      publish_once_ ? "true" : "false");

    publish_();

    if (!publish_once_ && publish_rate_hz_ > 0.0) {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&EsdfSliceMapPublisherNode::publish_, this));
    }
  }

private:
  void publish_()
  {
    const auto stamp = now();
    if (publish_occupancy_ && pub_) {
      map_.header.stamp = stamp;
      pub_->publish(map_);
    }
    if (publish_pointcloud_ && pointcloud_pub_) {
      cloud_.header.stamp = stamp;
      pointcloud_pub_->publish(cloud_);
    }
  }

  std::string map_yaml_path_;
  std::string topic_;
  std::string pointcloud_topic_;
  std::string frame_id_;
  bool publish_occupancy_{true};
  bool publish_pointcloud_{true};
  bool publish_once_{true};
  int pointcloud_stride_{1};
  double max_pointcloud_distance_m_{std::numeric_limits<double>::infinity()};
  double slice_height_m_{0.0};
  double publish_rate_hz_{1.0};
  nav_msgs::msg::OccupancyGrid map_;
  cv::Mat distance_meters_;
  sensor_msgs::msg::PointCloud2 cloud_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<EsdfSliceMapPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("esdf_slice_map_publisher"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
