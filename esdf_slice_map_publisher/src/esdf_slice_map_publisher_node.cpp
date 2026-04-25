#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace
{

struct GrayImage
{
  int width{0};
  int height{0};
  int max_value{255};
  std::vector<uint8_t> pixels;
};

struct MapSpec
{
  std::filesystem::path image_path;
  std::filesystem::path yaml_path;
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  int negate{0};
  double occupied_thresh{0.65};
  double free_thresh{0.196};
  std::string mode{"trinary"};
};

std::string readNextPgmToken(std::istream & stream)
{
  while (true) {
    const int next = stream.peek();
    if (next == EOF) {
      throw std::runtime_error("unexpected EOF while reading PGM header");
    }
    if (std::isspace(next)) {
      stream.get();
      continue;
    }
    if (next == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    break;
  }

  std::string token;
  while (true) {
    const int next = stream.peek();
    if (next == EOF || std::isspace(next) || next == '#') {
      break;
    }
    token.push_back(static_cast<char>(stream.get()));
  }

  if (token.empty()) {
    throw std::runtime_error("failed to read token from PGM");
  }
  return token;
}

void skipPgmWhitespaceAndComments(std::istream & stream)
{
  while (true) {
    const int next = stream.peek();
    if (next == EOF) {
      return;
    }
    if (std::isspace(next)) {
      stream.get();
      continue;
    }
    if (next == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    return;
  }
}

GrayImage loadPgm(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::in | std::ios::binary);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open image: " + path.string());
  }

  const std::string magic = readNextPgmToken(stream);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("unsupported PGM format '" + magic + "' in " + path.string());
  }

  GrayImage image;
  image.width = std::stoi(readNextPgmToken(stream));
  image.height = std::stoi(readNextPgmToken(stream));
  image.max_value = std::stoi(readNextPgmToken(stream));
  if (image.width <= 0 || image.height <= 0) {
    throw std::runtime_error("invalid PGM dimensions in " + path.string());
  }
  if (image.max_value <= 0 || image.max_value > 255) {
    throw std::runtime_error("unsupported PGM max value in " + path.string());
  }

  const size_t pixel_count = static_cast<size_t>(image.width) * static_cast<size_t>(image.height);
  image.pixels.resize(pixel_count);

  if (magic == "P5") {
    skipPgmWhitespaceAndComments(stream);
    stream.read(reinterpret_cast<char *>(image.pixels.data()), static_cast<std::streamsize>(pixel_count));
    if (stream.gcount() != static_cast<std::streamsize>(pixel_count)) {
      throw std::runtime_error("unexpected EOF while reading PGM raster from " + path.string());
    }
  } else {
    for (size_t i = 0; i < pixel_count; ++i) {
      const int value = std::stoi(readNextPgmToken(stream));
      image.pixels[i] = static_cast<uint8_t>(std::clamp(value, 0, image.max_value));
    }
  }

  return image;
}

MapSpec loadMapSpec(const std::filesystem::path & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path.string());
  MapSpec spec;
  spec.yaml_path = std::filesystem::weakly_canonical(yaml_path);

  const auto image_field = root["image"];
  const auto resolution_field = root["resolution"];
  const auto origin_field = root["origin"];
  if (!image_field || !resolution_field || !origin_field || !origin_field.IsSequence() ||
    origin_field.size() < 3)
  {
    throw std::runtime_error("map YAML is missing required fields");
  }

  std::filesystem::path image_path = image_field.as<std::string>();
  if (image_path.is_relative()) {
    image_path = yaml_path.parent_path() / image_path;
  }

  spec.image_path = std::filesystem::weakly_canonical(image_path);
  spec.resolution = resolution_field.as<double>();
  spec.origin_x = origin_field[0].as<double>();
  spec.origin_y = origin_field[1].as<double>();
  spec.origin_yaw = origin_field[2].as<double>();
  if (root["negate"]) {
    spec.negate = root["negate"].as<int>();
  }
  if (root["occupied_thresh"]) {
    spec.occupied_thresh = root["occupied_thresh"].as<double>();
  }
  if (root["free_thresh"]) {
    spec.free_thresh = root["free_thresh"].as<double>();
  }
  if (root["mode"]) {
    spec.mode = root["mode"].as<std::string>();
  }

  if (spec.resolution <= 0.0) {
    throw std::runtime_error("map resolution must be positive");
  }

  return spec;
}

std::optional<double> parseSliceHeightFromPath(const std::filesystem::path & path)
{
  const std::string stem = path.stem().string();
  const std::string needle = "esdf_slice_z_";
  const size_t start = stem.find(needle);
  if (start == std::string::npos) {
    return std::nullopt;
  }

  const size_t value_start = start + needle.size();
  const size_t value_end = stem.find("_occupancy", value_start);
  const std::string encoded = stem.substr(value_start, value_end - value_start);
  if (encoded.empty()) {
    return std::nullopt;
  }

  for (size_t dot_idx = encoded.find('_'); dot_idx != std::string::npos; dot_idx = encoded.find('_', dot_idx + 1)) {
    std::string candidate = encoded;
    candidate[dot_idx] = '.';
    try {
      return std::stod(candidate);
    } catch (const std::exception &) {
    }
  }

  try {
    return std::stod(encoded);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

int8_t probabilityToOccupancy(double probability, const MapSpec & spec)
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

nav_msgs::msg::OccupancyGrid buildOccupancyGrid(
  const GrayImage & image,
  const MapSpec & spec,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
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

cv::Mat buildDistanceTransformMeters(const nav_msgs::msg::OccupancyGrid & map)
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
  const nav_msgs::msg::OccupancyGrid & map,
  const cv::Mat & distance_meters,
  const std::string & frame_id,
  double slice_height_m,
  const rclcpp::Time & stamp)
{
  size_t point_count = 0;
  for (const int8_t cell : map.data) {
    if (cell >= 0) {
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

  for (uint32_t y = 0; y < map.info.height; ++y) {
    for (uint32_t x = 0; x < map.info.width; ++x) {
      const size_t index = static_cast<size_t>(y * map.info.width + x);
      const int8_t cell = map.data[index];
      if (cell < 0) {
        continue;
      }

      *iter_x = static_cast<float>(origin_x + (static_cast<double>(x) + 0.5) * resolution);
      *iter_y = static_cast<float>(origin_y + (static_cast<double>(y) + 0.5) * resolution);
      *iter_z = static_cast<float>(slice_height_m);
      *iter_intensity = distance_meters.at<float>(static_cast<int>(y), static_cast<int>(x));

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  return cloud;
}

}  // namespace

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
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    const double configured_slice_height_m = declare_parameter<double>("slice_height_m", std::numeric_limits<double>::quiet_NaN());
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

    if (map_yaml_path_.empty()) {
      throw std::runtime_error("map_yaml_path parameter is required");
    }

    const auto spec = loadMapSpec(map_yaml_path_);
    const auto image = loadPgm(spec.image_path);
    map_ = buildOccupancyGrid(image, spec, frame_id_, now());
    if (std::isfinite(configured_slice_height_m)) {
      slice_height_m_ = configured_slice_height_m;
    } else {
      slice_height_m_ = parseSliceHeightFromPath(spec.yaml_path).value_or(0.0);
    }
    distance_meters_ = buildDistanceTransformMeters(map_);
    cloud_ = buildDistancePointCloud(map_, distance_meters_, frame_id_, slice_height_m_, now());

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);
    if (publish_pointcloud_) {
      pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, qos);
    }

    const std::string pointcloud_suffix = publish_pointcloud_ ?
      (" and pointcloud on '" + pointcloud_topic_ + "'") :
      "";
    RCLCPP_INFO(
      get_logger(),
      "Loaded map '%s' (%ux%u at %.3f m/cell), publishing occupancy on '%s'%s",
      map_yaml_path_.c_str(),
      map_.info.width,
      map_.info.height,
      map_.info.resolution,
      topic_.c_str(),
      pointcloud_suffix.c_str());

    publish_();

    if (publish_rate_hz_ > 0.0) {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&EsdfSliceMapPublisherNode::publish_, this));
    }
  }

private:
  void publish_()
  {
    const auto stamp = now();
    map_.header.stamp = stamp;
    pub_->publish(map_);
    if (publish_pointcloud_ && pointcloud_pub_) {
      cloud_.header.stamp = stamp;
      pointcloud_pub_->publish(cloud_);
    }
  }

  std::string map_yaml_path_;
  std::string topic_;
  std::string pointcloud_topic_;
  std::string frame_id_;
  bool publish_pointcloud_{true};
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
