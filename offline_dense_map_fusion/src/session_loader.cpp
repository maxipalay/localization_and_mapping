#include "offline_dense_map_fusion/session_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace offline_dense_map_fusion
{

namespace
{

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
    throw std::runtime_error("CSV row has different column count than header");
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
    throw std::runtime_error("missing required CSV field '" + key + "'");
  }
  return it->second;
}

uint64_t parseUint64(const std::string &value, const std::string &name)
{
  try {
    return static_cast<uint64_t>(std::stoull(value));
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse integer field '" + name + "'");
  }
}

int64_t parseInt64(const std::string &value, const std::string &name)
{
  try {
    return static_cast<int64_t>(std::stoll(value));
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse integer field '" + name + "'");
  }
}

double parseDouble(const std::string &value, const std::string &name)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse floating field '" + name + "'");
  }
}

Pose makePose(
  double px,
  double py,
  double pz,
  double qx,
  double qy,
  double qz,
  double qw)
{
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm <= 0.0) {
    throw std::runtime_error("invalid zero-norm quaternion");
  }
  const double x = qx / norm;
  const double y = qy / norm;
  const double z = qz / norm;
  const double w = qw / norm;

  Pose pose;
  pose.rotation = cv::Matx33d(
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y));
  pose.translation = cv::Vec3d(px, py, pz);
  return pose;
}

Pose parsePoseColumns(const CsvRow &row, const std::string &prefix)
{
  return makePose(
    parseDouble(requireField(row, prefix + "px"), prefix + "px"),
    parseDouble(requireField(row, prefix + "py"), prefix + "py"),
    parseDouble(requireField(row, prefix + "pz"), prefix + "pz"),
    parseDouble(requireField(row, prefix + "qx"), prefix + "qx"),
    parseDouble(requireField(row, prefix + "qy"), prefix + "qy"),
    parseDouble(requireField(row, prefix + "qz"), prefix + "qz"),
    parseDouble(requireField(row, prefix + "qw"), prefix + "qw"));
}

Pose parsePoseYamlNode(const YAML::Node &node)
{
  const YAML::Node position = node["position"];
  const YAML::Node quat = node["orientation_xyzw"];
  if (!position || !quat || !position.IsSequence() || !quat.IsSequence() ||
      position.size() != 3 || quat.size() != 4) {
    throw std::runtime_error("pose YAML must contain position[3] and orientation_xyzw[4]");
  }

  return makePose(
    position[0].as<double>(),
    position[1].as<double>(),
    position[2].as<double>(),
    quat[0].as<double>(),
    quat[1].as<double>(),
    quat[2].as<double>(),
    quat[3].as<double>());
}

std::filesystem::path resolveRelativePath(const std::filesystem::path &base, const std::string &value)
{
  if (value.empty()) {
    return {};
  }
  return base / value;
}

CameraIntrinsics loadCamera(const std::filesystem::path &path)
{
  const YAML::Node root = YAML::LoadFile(path.string());
  CameraIntrinsics camera;
  camera.frame_id = root["header_frame_id"].as<std::string>();
  camera.width = root["width"].as<int>();
  camera.height = root["height"].as<int>();

  const YAML::Node p = root["p"];
  if (p && p.IsSequence() && p.size() == 12) {
    camera.fx = p[0].as<double>();
    camera.fy = p[5].as<double>();
    camera.cx = p[2].as<double>();
    camera.cy = p[6].as<double>();
  } else {
    const YAML::Node k = root["k"];
    if (!k || !k.IsSequence() || k.size() != 9) {
      throw std::runtime_error("camera info YAML must contain either p[12] or k[9]");
    }
    camera.fx = k[0].as<double>();
    camera.fy = k[4].as<double>();
    camera.cx = k[2].as<double>();
    camera.cy = k[5].as<double>();
  }

  return camera;
}

Pose compose(const Pose &lhs, const Pose &rhs)
{
  Pose out;
  out.rotation = lhs.rotation * rhs.rotation;
  out.translation = lhs.rotation * rhs.translation + lhs.translation;
  return out;
}

}  // namespace

SessionData loadSessionWithOptimizedPoses(const std::filesystem::path &session_dir)
{
  SessionData session;
  session.session_dir = session_dir;
  session.session_name = session_dir.filename().string();

  const auto metadata_path = session_dir / "session_metadata.yaml";
  if (std::filesystem::exists(metadata_path)) {
    const YAML::Node metadata = YAML::LoadFile(metadata_path.string());
    if (metadata["session_name"]) {
      session.session_name = metadata["session_name"].as<std::string>();
    }
    const YAML::Node frames = metadata["frames"];
    if (frames && frames["body"]) {
      session.body_frame_id = frames["body"].as<std::string>();
    }
  }

  session.camera = loadCamera(session_dir / "calibration" / "rgb_camera_info.yaml");

  std::unordered_map<uint64_t, Pose> optimized_body_poses;
  {
    std::ifstream is(session_dir / "offline_global_graph" / "optimized_keyframes.csv");
    if (!is.is_open()) {
      throw std::runtime_error(
        "failed to open optimized poses. Run offline_global_graph first for session: " +
        session_dir.string());
    }

    std::string line;
    if (!std::getline(is, line)) {
      throw std::runtime_error("optimized_keyframes.csv is empty");
    }
    const auto header = parseCsvLine(line);

    while (std::getline(is, line)) {
      if (trim(line).empty()) {
        continue;
      }
      const auto row = makeCsvRow(header, parseCsvLine(line));
      const uint64_t kf_id = parseUint64(requireField(row, "kf_id"), "kf_id");
      optimized_body_poses.emplace(kf_id, parsePoseColumns(row, "opt_body_"));
    }
  }

  {
    std::ifstream is(session_dir / "keyframe_manifest.csv");
    if (!is.is_open()) {
      throw std::runtime_error("failed to open keyframe manifest");
    }

    std::string line;
    if (!std::getline(is, line)) {
      throw std::runtime_error("keyframe manifest is empty");
    }
    const auto header = parseCsvLine(line);

    while (std::getline(is, line)) {
      if (trim(line).empty()) {
        continue;
      }

      const auto row = makeCsvRow(header, parseCsvLine(line));
      const uint64_t kf_id = parseUint64(requireField(row, "kf_id"), "kf_id");
      const auto pose_it = optimized_body_poses.find(kf_id);
      if (pose_it == optimized_body_poses.end()) {
        continue;
      }

      const auto depth_path = resolveRelativePath(session_dir, requireField(row, "depth_path"));
      const auto rgb_path = resolveRelativePath(session_dir, requireField(row, "rgb_path"));
      if (depth_path.empty() || rgb_path.empty()) {
        continue;
      }

      SessionFrame frame;
      frame.kf_id = kf_id;
      frame.stamp_ns = parseInt64(requireField(row, "keyframe_stamp_ns"), "keyframe_stamp_ns");
      frame.rgb_path = rgb_path;
      frame.depth_path = depth_path;
      frame.world_T_body = pose_it->second;
      session.frames.push_back(std::move(frame));
    }
  }

  std::sort(
    session.frames.begin(),
    session.frames.end(),
    [](const SessionFrame &lhs, const SessionFrame &rhs) {
      if (lhs.stamp_ns == rhs.stamp_ns) {
        return lhs.kf_id < rhs.kf_id;
      }
      return lhs.stamp_ns < rhs.stamp_ns;
    });

  if (session.frames.empty()) {
    throw std::runtime_error("no session frames with optimized poses and RGB-D paths were found");
  }

  return session;
}

Extrinsics loadExtrinsics(const std::filesystem::path &path)
{
  const YAML::Node root = YAML::LoadFile(path.string());
  Extrinsics extrinsics;
  if (root["body_frame_id"]) {
    extrinsics.body_frame_id = root["body_frame_id"].as<std::string>();
  }
  if (root["camera_frame_id"]) {
    extrinsics.camera_frame_id = root["camera_frame_id"].as<std::string>();
  }
  const YAML::Node pose_node = root["body_T_camera"] ? root["body_T_camera"] : root;
  extrinsics.body_T_camera = parsePoseYamlNode(pose_node);
  return extrinsics;
}

}  // namespace offline_dense_map_fusion
