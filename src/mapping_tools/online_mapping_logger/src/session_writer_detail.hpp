#pragma once

#include "online_mapping_logger/session_writer.hpp"

#include <filesystem>
#include <ostream>
#include <string>

namespace online_mapping_logger::detail
{

template <typename TContainer>
inline void writeScalarArray(std::ostream &os, const std::string &key, const TContainer &values)
{
  os << key << ": [";
  size_t i = 0;
  for (const auto &value : values) {
    if (i > 0) {
      os << ", ";
    }
    os << +value;
    ++i;
  }
  os << "]\n";
}

inline void writePoseYaml(
  std::ostream &os,
  const std::string &key,
  const geometry_msgs::msg::Pose &pose)
{
  os << key << ":\n";
  os << "  position: [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
     << "]\n";
  os << "  orientation_xyzw: ["
     << pose.orientation.x << ", "
     << pose.orientation.y << ", "
     << pose.orientation.z << ", "
     << pose.orientation.w << "]\n";
}

inline void writeVector3Yaml(
  std::ostream &os,
  const std::string &key,
  const geometry_msgs::msg::Vector3 &vector)
{
  os << key << ": ["
     << vector.x << ", "
     << vector.y << ", "
     << vector.z << "]\n";
}

inline void writeIntervalHealthYaml(
  std::ostream &os,
  const visual_inertial::msg::FrontendIntervalHealth &health)
{
  os << "interval_health:\n";
  os << "  num_frames: " << health.num_frames << "\n";
  os << "  num_pose_valid_frames: " << health.num_pose_valid_frames << "\n";
  os << "  num_degraded_frames: " << health.num_degraded_frames << "\n";
  os << "  num_lost_frames: " << health.num_lost_frames << "\n";
  os << "  min_tracks: " << health.min_tracks << "\n";
  os << "  mean_tracks: " << health.mean_tracks << "\n";
  os << "  min_track_retention: " << health.min_track_retention << "\n";
  os << "  mean_track_retention: " << health.mean_track_retention << "\n";
  os << "  mean_pnp_inlier_ratio: " << health.mean_pnp_inlier_ratio << "\n";
  os << "  max_pnp_reproj_rmse_px: " << health.max_pnp_reproj_rmse_px << "\n";
  os << "  min_track_coverage: " << health.min_track_coverage << "\n";
  os << "  mean_track_coverage: " << health.mean_track_coverage << "\n";
}

inline std::string csvEscape(const std::string &value)
{
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }

  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(c);
    }
  }
  escaped.push_back('"');
  return escaped;
}

inline std::string yamlDoubleQuoted(const std::string &value)
{
  std::string escaped = "\"";
  escaped.reserve(value.size() + 2);
  for (const char c : value) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped.push_back(c);
        break;
    }
  }
  escaped.push_back('"');
  return escaped;
}

inline std::filesystem::path relativeTo(
  const std::filesystem::path &path,
  const std::filesystem::path &base)
{
  std::error_code ec;
  auto rel = std::filesystem::relative(path, base, ec);
  return ec ? path.filename() : rel;
}

}  // namespace online_mapping_logger::detail
