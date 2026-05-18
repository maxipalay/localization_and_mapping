#include "map_conversion.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace esdf_slice_map_publisher
{

namespace
{

int8_t probabilityToOccupancy(double probability, const MapSpec &spec)
{
  if (spec.mode == "trinary") {
    if (probability > spec.occupied_thresh) {
      return 100;
    }
    if (probability < spec.free_thresh) {
      return 0;
    }
    return -1;
  }

  if (spec.mode == "scale") {
    if (probability > spec.occupied_thresh) {
      return 100;
    }
    if (probability < spec.free_thresh) {
      return 0;
    }
    const double scaled =
      (probability - spec.free_thresh) / std::max(1e-9, spec.occupied_thresh - spec.free_thresh);
    return static_cast<int8_t>(std::clamp(std::lround(scaled * 99.0), 1L, 99L));
  }

  if (spec.mode == "raw") {
    const long raw = std::lround(probability * 100.0);
    return static_cast<int8_t>(std::clamp(raw, 0L, 100L));
  }

  throw std::runtime_error("unsupported map mode: " + spec.mode);
}

}  // namespace

nav_msgs::msg::OccupancyGrid buildOccupancyGrid(
  const GrayImage &image,
  const MapSpec &spec,
  const std::string &frame_id,
  const rclcpp::Time &stamp)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.stamp = stamp;
  map.header.frame_id = frame_id;
  map.info.resolution = static_cast<float>(spec.resolution);
  map.info.width = static_cast<uint32_t>(image.width);
  map.info.height = static_cast<uint32_t>(image.height);
  map.info.origin.position.x = spec.origin_x;
  map.info.origin.position.y = spec.origin_y;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = std::sin(spec.origin_yaw * 0.5);
  map.info.origin.orientation.w = std::cos(spec.origin_yaw * 0.5);
  map.data.resize(static_cast<size_t>(image.width) * static_cast<size_t>(image.height));

  for (int y = 0; y < image.height; ++y) {
    for (int x = 0; x < image.width; ++x) {
      const size_t src_index =
        static_cast<size_t>((image.height - 1 - y) * image.width + x);
      const uint8_t value = image.pixels[src_index];
      double normalized = static_cast<double>(value) / static_cast<double>(image.max_value);
      if (spec.negate != 0) {
        normalized = 1.0 - normalized;
      }
      const double occupancy_probability = 1.0 - normalized;
      map.data[static_cast<size_t>(y * image.width + x)] =
        probabilityToOccupancy(occupancy_probability, spec);
    }
  }

  return map;
}

cv::Mat buildDistanceTransformMeters(const nav_msgs::msg::OccupancyGrid &map)
{
  cv::Mat source(
    static_cast<int>(map.info.height),
    static_cast<int>(map.info.width),
    CV_8UC1,
    cv::Scalar(255));

  for (uint32_t y = 0; y < map.info.height; ++y) {
    for (uint32_t x = 0; x < map.info.width; ++x) {
      const int8_t cell = map.data[static_cast<size_t>(y * map.info.width + x)];
      if (cell == 100) {
        source.at<uint8_t>(static_cast<int>(y), static_cast<int>(x)) = 0;
      }
    }
  }

  cv::Mat distance_pixels;
  cv::distanceTransform(source, distance_pixels, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  distance_pixels *= static_cast<float>(map.info.resolution);
  return distance_pixels;
}

sensor_msgs::msg::PointCloud2 buildDistancePointCloud(
  const nav_msgs::msg::OccupancyGrid &map,
  const cv::Mat &distance_meters,
  const std::string &frame_id,
  double slice_height_m,
  int pointcloud_stride,
  double max_pointcloud_distance_m,
  const rclcpp::Time &stamp)
{
  size_t point_count = 0;
  for (uint32_t y = 0; y < map.info.height; y += static_cast<uint32_t>(pointcloud_stride)) {
    for (uint32_t x = 0; x < map.info.width; x += static_cast<uint32_t>(pointcloud_stride)) {
      const size_t index = static_cast<size_t>(y * map.info.width + x);
      const int8_t cell = map.data[index];
      if (cell < 0) {
        continue;
      }
      const float distance = distance_meters.at<float>(static_cast<int>(y), static_cast<int>(x));
      if (std::isfinite(max_pointcloud_distance_m) &&
          distance > static_cast<float>(max_pointcloud_distance_m))
      {
        continue;
      }
      ++point_count;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  for (uint32_t y = 0; y < map.info.height; y += static_cast<uint32_t>(pointcloud_stride)) {
    for (uint32_t x = 0; x < map.info.width; x += static_cast<uint32_t>(pointcloud_stride)) {
      const size_t index = static_cast<size_t>(y * map.info.width + x);
      const int8_t cell = map.data[index];
      if (cell < 0) {
        continue;
      }
      const float distance = distance_meters.at<float>(static_cast<int>(y), static_cast<int>(x));
      if (std::isfinite(max_pointcloud_distance_m) &&
          distance > static_cast<float>(max_pointcloud_distance_m))
      {
        continue;
      }

      *iter_x = static_cast<float>(origin_x + (static_cast<double>(x) + 0.5) * resolution);
      *iter_y = static_cast<float>(origin_y + (static_cast<double>(y) + 0.5) * resolution);
      *iter_z = static_cast<float>(slice_height_m);
      *iter_intensity = distance;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  return cloud;
}

}  // namespace esdf_slice_map_publisher
