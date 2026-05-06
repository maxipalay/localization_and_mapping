#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

struct PoseRecord
{
  std::string frame_name;
  uint64_t id{0};
  double px{0.0};
  double py{0.0};
  double pz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  int observation_count{0};
};

using CsvRow = std::unordered_map<std::string, std::string>;

std::string trim(const std::string &input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::vector<std::string> parseCsvLine(const std::string &line)
{
  std::vector<std::string> out;
  std::string current;
  bool in_quotes = false;

  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (in_quotes && (i + 1) < line.size() && line[i + 1] == '"') {
        current.push_back('"');
        ++i;
      } else {
        in_quotes = !in_quotes;
      }
    } else if (c == ',' && !in_quotes) {
      out.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }

  out.push_back(current);
  return out;
}

CsvRow makeCsvRow(const std::vector<std::string> &header, const std::vector<std::string> &values)
{
  if (header.size() != values.size()) {
    throw std::runtime_error("CSV row has a different column count than the header");
  }

  CsvRow row;
  for (size_t i = 0; i < header.size(); ++i) {
    row.emplace(header[i], values[i]);
  }
  return row;
}

const std::string &requireField(const CsvRow &row, const std::string &key)
{
  const auto it = row.find(key);
  if (it == row.end()) {
    throw std::runtime_error("missing required field '" + key + "'");
  }
  return it->second;
}

double parseDouble(const std::string &value, const std::string &name)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse field '" + name + "'");
  }
}

uint64_t parseUint64(const std::string &value, const std::string &name)
{
  try {
    return static_cast<uint64_t>(std::stoull(value));
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse field '" + name + "'");
  }
}

std::vector<PoseRecord> loadKeyframes(
  const std::string &session_dir,
  const std::string &prefix)
{
  std::ifstream is(session_dir + "/offline_global_graph/optimized_keyframes.csv");
  if (!is.is_open()) {
    throw std::runtime_error("failed to open optimized_keyframes.csv");
  }

  std::string line;
  if (!std::getline(is, line)) {
    throw std::runtime_error("optimized_keyframes.csv is empty");
  }

  const auto header = parseCsvLine(line);
  std::vector<PoseRecord> keyframes;
  while (std::getline(is, line)) {
    if (trim(line).empty()) {
      continue;
    }
    const auto row = makeCsvRow(header, parseCsvLine(line));
    PoseRecord record;
    record.id = parseUint64(requireField(row, "kf_id"), "kf_id");
    record.frame_name = prefix + std::to_string(record.id);
    record.px = parseDouble(requireField(row, "opt_body_px"), "opt_body_px");
    record.py = parseDouble(requireField(row, "opt_body_py"), "opt_body_py");
    record.pz = parseDouble(requireField(row, "opt_body_pz"), "opt_body_pz");
    record.qx = parseDouble(requireField(row, "opt_body_qx"), "opt_body_qx");
    record.qy = parseDouble(requireField(row, "opt_body_qy"), "opt_body_qy");
    record.qz = parseDouble(requireField(row, "opt_body_qz"), "opt_body_qz");
    record.qw = parseDouble(requireField(row, "opt_body_qw"), "opt_body_qw");
    keyframes.push_back(std::move(record));
  }

  return keyframes;
}

std::vector<PoseRecord> loadTags(
  const std::string &session_dir,
  const std::string &prefix)
{
  const YAML::Node root = YAML::LoadFile(session_dir + "/offline_global_graph/optimized_tags.yaml");
  const YAML::Node tags = root["tags"];
  std::vector<PoseRecord> out;
  if (!tags || !tags.IsSequence()) {
    return out;
  }

  for (const auto &tag : tags) {
    PoseRecord record;
    record.id = tag["id"].as<uint64_t>();
    record.frame_name = prefix + std::to_string(record.id);
    record.observation_count = tag["observation_count"] ? tag["observation_count"].as<int>() : 0;
    const auto position = tag["position"];
    const auto quat = tag["orientation_xyzw"];
    record.px = position[0].as<double>();
    record.py = position[1].as<double>();
    record.pz = position[2].as<double>();
    record.qx = quat[0].as<double>();
    record.qy = quat[1].as<double>();
    record.qz = quat[2].as<double>();
    record.qw = quat[3].as<double>();
    out.push_back(std::move(record));
  }

  return out;
}

geometry_msgs::msg::TransformStamped toTransform(
  const PoseRecord &record,
  const std::string &world_frame_id,
  const rclcpp::Time &stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = world_frame_id;
  tf.child_frame_id = record.frame_name;
  tf.transform.translation.x = record.px;
  tf.transform.translation.y = record.py;
  tf.transform.translation.z = record.pz;
  tf.transform.rotation.x = record.qx;
  tf.transform.rotation.y = record.qy;
  tf.transform.rotation.z = record.qz;
  tf.transform.rotation.w = record.qw;
  return tf;
}

geometry_msgs::msg::Point toPoint(const PoseRecord &record)
{
  geometry_msgs::msg::Point point;
  point.x = record.px;
  point.y = record.py;
  point.z = record.pz;
  return point;
}

class OfflineGlobalGraphVizNode final : public rclcpp::Node
{
public:
  OfflineGlobalGraphVizNode()
  : Node("offline_global_graph_viz")
  {
    session_dir_ = declare_parameter<std::string>("session_dir", "");
    world_frame_id_ = declare_parameter<std::string>("world_frame_id", "map");
    keyframe_prefix_ = declare_parameter<std::string>("keyframe_prefix", "optimized_kf_");
    tag_prefix_ = declare_parameter<std::string>("tag_prefix", "optimized_tag_");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    if (session_dir_.empty()) {
      throw std::runtime_error("session_dir parameter is required");
    }

    keyframes_ = loadKeyframes(session_dir_, keyframe_prefix_);
    tags_ = loadTags(session_dir_, tag_prefix_);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "offline_global_graph/markers",
      rclcpp::QoS(1).transient_local().reliable());

    if (publish_tf_) {
      static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      publishStaticTransforms_();
    }

    publishMarkers_();
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OfflineGlobalGraphVizNode::publishMarkers_, this));
  }

private:
  void publishStaticTransforms_()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(keyframes_.size() + tags_.size());
    const auto stamp = now();

    for (const auto &keyframe : keyframes_) {
      transforms.push_back(toTransform(keyframe, world_frame_id_, stamp));
    }
    for (const auto &tag : tags_) {
      transforms.push_back(toTransform(tag, world_frame_id_, stamp));
    }

    static_tf_broadcaster_->sendTransform(transforms);
  }

  visualization_msgs::msg::Marker makeMarkerBase(
    int32_t id,
    const std::string &ns,
    int32_t type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration(0, 0);
    return marker;
  }

  void publishMarkers_()
  {
    visualization_msgs::msg::MarkerArray array;

    auto path = makeMarkerBase(0, "keyframe_path", visualization_msgs::msg::Marker::LINE_STRIP);
    path.scale.x = 0.03;
    path.color.r = 0.2f;
    path.color.g = 0.8f;
    path.color.b = 1.0f;
    path.color.a = 1.0f;
    for (const auto &keyframe : keyframes_) {
      path.points.push_back(toPoint(keyframe));
    }
    array.markers.push_back(path);

    int32_t next_id = 1;
    for (const auto &keyframe : keyframes_) {
      auto marker = makeMarkerBase(next_id++, "keyframes", visualization_msgs::msg::Marker::SPHERE);
      marker.pose.position = toPoint(keyframe);
      marker.scale.x = 0.08;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;
      marker.color.r = 0.1f;
      marker.color.g = 0.9f;
      marker.color.b = 0.9f;
      marker.color.a = 0.9f;
      array.markers.push_back(marker);

      auto text = makeMarkerBase(next_id++, "keyframe_labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text.pose.position = toPoint(keyframe);
      text.pose.position.z += 0.12;
      text.scale.z = 0.12;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = "kf_" + std::to_string(keyframe.id);
      array.markers.push_back(text);
    }

    for (const auto &tag : tags_) {
      auto cube = makeMarkerBase(next_id++, "tags", visualization_msgs::msg::Marker::CUBE);
      cube.pose.position = toPoint(tag);
      cube.pose.orientation.x = tag.qx;
      cube.pose.orientation.y = tag.qy;
      cube.pose.orientation.z = tag.qz;
      cube.pose.orientation.w = tag.qw;
      cube.scale.x = 0.18;
      cube.scale.y = 0.18;
      cube.scale.z = 0.02;
      cube.color.r = 1.0f;
      cube.color.g = 0.7f;
      cube.color.b = 0.1f;
      cube.color.a = 0.95f;
      array.markers.push_back(cube);

      auto text = makeMarkerBase(next_id++, "tag_labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text.pose.position = toPoint(tag);
      text.pose.position.z += 0.18;
      text.scale.z = 0.14;
      text.color.r = 1.0f;
      text.color.g = 0.95f;
      text.color.b = 0.8f;
      text.color.a = 1.0f;
      text.text = "tag_" + std::to_string(tag.id) + " (" + std::to_string(tag.observation_count) + ")";
      array.markers.push_back(text);
    }

    marker_pub_->publish(array);
  }

  std::string session_dir_;
  std::string world_frame_id_;
  std::string keyframe_prefix_;
  std::string tag_prefix_;
  bool publish_tf_{true};
  std::vector<PoseRecord> keyframes_;
  std::vector<PoseRecord> tags_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OfflineGlobalGraphVizNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
