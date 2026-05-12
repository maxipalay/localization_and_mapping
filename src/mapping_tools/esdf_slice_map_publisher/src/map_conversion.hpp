#pragma once

#include "map_io.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/core/mat.hpp>

#include <string>

namespace esdf_slice_map_publisher
{

nav_msgs::msg::OccupancyGrid buildOccupancyGrid(
  const GrayImage &image,
  const MapSpec &spec,
  const std::string &frame_id,
  const rclcpp::Time &stamp);

cv::Mat buildDistanceTransformMeters(const nav_msgs::msg::OccupancyGrid &map);

sensor_msgs::msg::PointCloud2 buildDistancePointCloud(
  const nav_msgs::msg::OccupancyGrid &map,
  const cv::Mat &distance_meters,
  const std::string &frame_id,
  double slice_height_m,
  int pointcloud_stride,
  double max_pointcloud_distance_m,
  const rclcpp::Time &stamp);

}  // namespace esdf_slice_map_publisher
