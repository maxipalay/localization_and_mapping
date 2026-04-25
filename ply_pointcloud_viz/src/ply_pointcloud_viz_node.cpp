#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

struct PlyHeader
{
  bool ascii{false};
  std::size_t vertex_count{0};
  std::vector<std::string> vertex_properties;
};

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t h = std::hash<int>{}(key.x);
    h ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

struct ColorAccumulator
{
  double r{0.0};
  double g{0.0};
  double b{0.0};
  double a{0.0};
  std::uint32_t count{0};
};

PlyHeader parseHeader(std::ifstream & stream)
{
  PlyHeader header;
  std::string line;
  bool in_vertex_element = false;

  while (std::getline(stream, line)) {
    if (line == "ply") {
      continue;
    }
    if (line.rfind("format ", 0) == 0) {
      header.ascii = (line.find("ascii") != std::string::npos);
      continue;
    }
    if (line.rfind("element vertex ", 0) == 0) {
      in_vertex_element = true;
      header.vertex_count = static_cast<std::size_t>(std::stoull(line.substr(15)));
      continue;
    }
    if (line.rfind("element ", 0) == 0) {
      in_vertex_element = false;
      continue;
    }
    if (in_vertex_element && line.rfind("property ", 0) == 0) {
      std::istringstream ss(line);
      std::string property_kw;
      std::string type_name;
      std::string property_name;
      ss >> property_kw >> type_name >> property_name;
      if (!property_name.empty()) {
        header.vertex_properties.push_back(property_name);
      }
      continue;
    }
    if (line == "end_header") {
      return header;
    }
  }

  throw std::runtime_error("PLY header missing end_header");
}

float packRgb(const std::uint8_t r, const std::uint8_t g, const std::uint8_t b)
{
  const std::uint32_t packed =
    (static_cast<std::uint32_t>(r) << 16) |
    (static_cast<std::uint32_t>(g) << 8) |
    static_cast<std::uint32_t>(b);
  float rgb = 0.0f;
  std::memcpy(&rgb, &packed, sizeof(float));
  return rgb;
}

}  // namespace

class PlyPointCloudVizNode final : public rclcpp::Node
{
public:
  PlyPointCloudVizNode()
  : rclcpp::Node("ply_pointcloud_viz_node")
  {
    ply_path_ = declare_parameter<std::string>("ply_path", "");
    topic_ = declare_parameter<std::string>("topic", "/ply_pointcloud_viz/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    voxel_size_m_ = declare_parameter<double>("voxel_size_m", 0.05);
    vertex_stride_ = std::max<int>(1, declare_parameter<int>("vertex_stride", 1));
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    alpha_ = std::clamp(declare_parameter<double>("alpha", 1.0), 0.0, 1.0);

    if (ply_path_.empty()) {
      throw std::runtime_error("ply_path must be set");
    }
    if (voxel_size_m_ <= 0.0) {
      throw std::runtime_error("voxel_size_m must be positive");
    }

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    message_ = loadAndVoxelize_(ply_path_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_)),
      std::bind(&PlyPointCloudVizNode::publish_, this));

    publish_();
  }

private:
  sensor_msgs::msg::PointCloud2 loadAndVoxelize_(const std::string & ply_path)
  {
    std::ifstream stream(ply_path);
    if (!stream) {
      throw std::runtime_error("failed to open PLY: " + ply_path);
    }

    const PlyHeader header = parseHeader(stream);
    if (!header.ascii) {
      throw std::runtime_error("only ascii PLY is supported");
    }

    int idx_x = -1;
    int idx_y = -1;
    int idx_z = -1;
    int idx_r = -1;
    int idx_g = -1;
    int idx_b = -1;
    for (std::size_t i = 0; i < header.vertex_properties.size(); ++i) {
      const std::string & name = header.vertex_properties[i];
      if (name == "x") {
        idx_x = static_cast<int>(i);
      } else if (name == "y") {
        idx_y = static_cast<int>(i);
      } else if (name == "z") {
        idx_z = static_cast<int>(i);
      } else if (name == "red") {
        idx_r = static_cast<int>(i);
      } else if (name == "green") {
        idx_g = static_cast<int>(i);
      } else if (name == "blue") {
        idx_b = static_cast<int>(i);
      }
    }
    if (idx_x < 0 || idx_y < 0 || idx_z < 0) {
      throw std::runtime_error("PLY vertex properties must include x/y/z");
    }

    std::unordered_map<VoxelKey, ColorAccumulator, VoxelKeyHash> voxels;
    voxels.reserve(header.vertex_count / static_cast<std::size_t>(vertex_stride_) + 1);

    std::string line;
    std::size_t kept_vertices = 0;
    for (std::size_t vertex_idx = 0; vertex_idx < header.vertex_count; ++vertex_idx) {
      if (!std::getline(stream, line)) {
        throw std::runtime_error("unexpected EOF while reading PLY vertices");
      }

      if (vertex_idx % static_cast<std::size_t>(vertex_stride_) != 0U) {
        continue;
      }

      std::istringstream ss(line);
      std::vector<std::string> fields;
      fields.reserve(header.vertex_properties.size());
      std::string token;
      while (ss >> token) {
        fields.push_back(token);
      }
      if (fields.size() < header.vertex_properties.size()) {
        continue;
      }

      const double x = std::stod(fields[idx_x]);
      const double y = std::stod(fields[idx_y]);
      const double z = std::stod(fields[idx_z]);

      const int vx = static_cast<int>(std::floor(x / voxel_size_m_));
      const int vy = static_cast<int>(std::floor(y / voxel_size_m_));
      const int vz = static_cast<int>(std::floor(z / voxel_size_m_));

      VoxelKey key{vx, vy, vz};
      auto & accum = voxels[key];
      accum.r += (idx_r >= 0) ? static_cast<double>(std::stoi(fields[idx_r])) / 255.0 : 0.8;
      accum.g += (idx_g >= 0) ? static_cast<double>(std::stoi(fields[idx_g])) / 255.0 : 0.8;
      accum.b += (idx_b >= 0) ? static_cast<double>(std::stoi(fields[idx_b])) / 255.0 : 0.8;
      accum.a += alpha_;
      accum.count += 1;
      kept_vertices += 1;
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(voxels.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_rgb(msg, "rgb");

    for (const auto & entry : voxels) {
      const VoxelKey & voxel = entry.first;
      const ColorAccumulator & accum = entry.second;
      const double count = std::max(1u, accum.count);

      *iter_x = static_cast<float>((static_cast<double>(voxel.x) + 0.5) * voxel_size_m_);
      *iter_y = static_cast<float>((static_cast<double>(voxel.y) + 0.5) * voxel_size_m_);
      *iter_z = static_cast<float>((static_cast<double>(voxel.z) + 0.5) * voxel_size_m_);

      const std::uint8_t r = static_cast<std::uint8_t>(
        std::clamp(std::lround((accum.r / count) * 255.0), 0L, 255L));
      const std::uint8_t g = static_cast<std::uint8_t>(
        std::clamp(std::lround((accum.g / count) * 255.0), 0L, 255L));
      const std::uint8_t b = static_cast<std::uint8_t>(
        std::clamp(std::lround((accum.b / count) * 255.0), 0L, 255L));
      *iter_rgb = packRgb(r, g, b);

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }

    RCLCPP_INFO(
      get_logger(),
      "Voxelized %zu kept vertices into %zu colored points at %.3f m voxel size (vertex_stride=%d)",
      kept_vertices,
      voxels.size(),
      voxel_size_m_,
      vertex_stride_);

    return msg;
  }

  void publish_()
  {
    message_.header.stamp = now();
    pub_->publish(message_);
  }

  std::string ply_path_;
  std::string topic_;
  std::string frame_id_;
  double voxel_size_m_{0.05};
  int vertex_stride_{1};
  double publish_rate_hz_{1.0};
  double alpha_{1.0};
  sensor_msgs::msg::PointCloud2 message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PlyPointCloudVizNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("ply_pointcloud_viz"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
