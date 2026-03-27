#include "offline_global_graph/session_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace offline_global_graph
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

CsvRow makeCsvRow(
  const std::vector<std::string> &header,
  const std::vector<std::string> &values)
{
  if (header.size() != values.size()) {
    throw std::runtime_error("manifest row has a different column count than the header");
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
    throw std::runtime_error("manifest is missing required field '" + key + "'");
  }
  return it->second;
}

std::string optionalField(const CsvRow &row, const std::string &key)
{
  const auto it = row.find(key);
  return it == row.end() ? std::string() : it->second;
}

std::string fieldAlias(const CsvRow &row, const std::vector<std::string> &candidates)
{
  for (const auto &candidate : candidates) {
    const auto it = row.find(candidate);
    if (it != row.end()) {
      return it->second;
    }
  }
  throw std::runtime_error("manifest is missing one of the required aliased fields");
}

double parseDouble(const std::string &value, const std::string &name)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse floating-point field '" + name + "' from '" + value + "'");
  }
}

uint64_t parseUint64(const std::string &value, const std::string &name)
{
  try {
    return static_cast<uint64_t>(std::stoull(value));
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse integer field '" + name + "' from '" + value + "'");
  }
}

int64_t parseInt64(const std::string &value, const std::string &name)
{
  try {
    return static_cast<int64_t>(std::stoll(value));
  } catch (const std::exception &) {
    throw std::runtime_error("failed to parse integer field '" + name + "' from '" + value + "'");
  }
}

gtsam::Pose3 makePose(
  double px,
  double py,
  double pz,
  double qx,
  double qy,
  double qz,
  double qw)
{
  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(qw, qx, qy, qz),
    gtsam::Point3(px, py, pz));
}

gtsam::Pose3 poseFromManifest(const CsvRow &row, const std::string &prefix)
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

gtsam::Pose3 poseFromManifestAliases(const CsvRow &row, const std::vector<std::string> &prefixes)
{
  for (const auto &prefix : prefixes) {
    if (row.find(prefix + "px") != row.end()) {
      return poseFromManifest(row, prefix);
    }
  }
  throw std::runtime_error("manifest is missing all pose column aliases for optimized pose");
}

std::filesystem::path resolveRelativePath(
  const std::filesystem::path &base,
  const std::string &value)
{
  if (value.empty()) {
    return {};
  }
  return base / value;
}

std::string nodeStringOr(const YAML::Node &node, const std::string &key, const std::string &fallback)
{
  const auto value = node[key];
  return value ? value.as<std::string>() : fallback;
}

double nodeDoubleOr(const YAML::Node &node, const std::string &key, double fallback)
{
  const auto value = node[key];
  return value ? value.as<double>() : fallback;
}

int64_t nodeInt64Or(const YAML::Node &node, const std::string &key, int64_t fallback)
{
  const auto value = node[key];
  return value ? value.as<int64_t>() : fallback;
}

bool nodeBoolOr(const YAML::Node &node, const std::string &key, bool fallback)
{
  const auto value = node[key];
  if (!value) {
    return fallback;
  }

  try {
    return value.as<bool>();
  } catch (const YAML::BadConversion &) {
  }

  try {
    return value.as<int>() != 0;
  } catch (const YAML::BadConversion &) {
  }

  try {
    const auto text = value.as<std::string>();
    if (text == "true" || text == "True" || text == "TRUE" || text == "1") {
      return true;
    }
    if (text == "false" || text == "False" || text == "FALSE" || text == "0") {
      return false;
    }
  } catch (const YAML::BadConversion &) {
  }

  throw std::runtime_error("failed to parse boolean field '" + key + "'");
}

gtsam::Pose3 poseFromYamlNode(const YAML::Node &node)
{
  const YAML::Node translation = node["position"] ? node["position"] : node["translation"];
  const YAML::Node quaternion = node["orientation_xyzw"];
  if (!translation || !quaternion || !translation.IsSequence() || !quaternion.IsSequence() ||
      translation.size() != 3 || quaternion.size() != 4) {
    throw std::runtime_error("pose node must contain position/translation[3] and orientation_xyzw[4]");
  }

  return makePose(
    translation[0].as<double>(),
    translation[1].as<double>(),
    translation[2].as<double>(),
    quaternion[0].as<double>(),
    quaternion[1].as<double>(),
    quaternion[2].as<double>(),
    quaternion[3].as<double>());
}

void loadManifest(SessionData &session)
{
  const auto manifest_path = session.session_dir / "keyframe_manifest.csv";
  std::ifstream is(manifest_path);
  if (!is.is_open()) {
    throw std::runtime_error("failed to open manifest: " + manifest_path.string());
  }

  std::string line;
  if (!std::getline(is, line)) {
    throw std::runtime_error("manifest is empty: " + manifest_path.string());
  }
  const auto header = parseCsvLine(line);

  while (std::getline(is, line)) {
    if (trim(line).empty()) {
      continue;
    }

    const auto values = parseCsvLine(line);
    const auto row = makeCsvRow(header, values);

    KeyframeRecord record;
    record.kf_id = parseUint64(requireField(row, "kf_id"), "kf_id");
    record.stamp_ns = parseInt64(requireField(row, "keyframe_stamp_ns"), "keyframe_stamp_ns");
    record.rgb_path = resolveRelativePath(session.session_dir, optionalField(row, "rgb_path"));
    record.depth_path = resolveRelativePath(session.session_dir, optionalField(row, "depth_path"));
    record.tags_path = resolveRelativePath(session.session_dir, optionalField(row, "tags_path"));
    record.keyframe_meta_path =
      resolveRelativePath(session.session_dir, optionalField(row, "keyframe_meta_path"));
    record.frontend_pose_wc = poseFromManifest(row, "frontend_");
    record.initial_pose_wb = poseFromManifestAliases(row, {"opt_body_", "opt_"});
    session.keyframes.push_back(std::move(record));
  }

  std::sort(
    session.keyframes.begin(),
    session.keyframes.end(),
    [](const KeyframeRecord &lhs, const KeyframeRecord &rhs) {
      if (lhs.stamp_ns == rhs.stamp_ns) {
        return lhs.kf_id < rhs.kf_id;
      }
      return lhs.stamp_ns < rhs.stamp_ns;
    });
}

void loadKeyframeMetadata(SessionData &session)
{
  for (auto &keyframe : session.keyframes) {
    if (keyframe.keyframe_meta_path.empty() || !std::filesystem::exists(keyframe.keyframe_meta_path)) {
      continue;
    }

    const YAML::Node root = YAML::LoadFile(keyframe.keyframe_meta_path.string());
    const bool has_vo_between = nodeBoolOr(root, "has_vo_between", false);
    if (!has_vo_between) {
      continue;
    }

    YAML::Node between_node = root["between_pose_prev_curr_body"];
    if (!between_node) {
      between_node = root["between_pose_prev_curr"];
    }
    if (!between_node) {
      continue;
    }

    keyframe.between_pose_prev_curr_body = poseFromYamlNode(between_node);
  }
}

void loadSessionMetadata(SessionData &session)
{
  const auto metadata_path = session.session_dir / "session_metadata.yaml";
  if (!std::filesystem::exists(metadata_path)) {
    return;
  }

  const YAML::Node root = YAML::LoadFile(metadata_path.string());
  if (root["session_name"]) {
    session.session_name = root["session_name"].as<std::string>();
  } else {
    session.session_name = session.session_dir.filename().string();
  }

  const YAML::Node frames = root["frames"];
  if (frames && frames["body"]) {
    session.body_frame_id = frames["body"].as<std::string>();
  }
}

void loadTagPriors(SessionData &session)
{
  const auto priors_path = session.session_dir / "tag_priors.yaml";
  if (!std::filesystem::exists(priors_path)) {
    return;
  }

  std::ifstream is(priors_path);
  if (!is.is_open()) {
    throw std::runtime_error("failed to open tag priors file: " + priors_path.string());
  }

  std::stringstream buffer;
  buffer << is.rdbuf();
  const std::string contents = trim(buffer.str());
  if (contents.empty() || contents[0] == '#') {
    return;
  }

  const YAML::Node root = YAML::Load(contents);
  YAML::Node sequence = root["tags"] ? root["tags"] : root["priors"];
  if (!sequence && root.IsSequence()) {
    sequence = root;
  }
  if (!sequence || !sequence.IsSequence()) {
    throw std::runtime_error("tag_priors.yaml must contain a sequence under 'tags' or 'priors'");
  }

  for (const auto &entry : sequence) {
    if (!entry["id"] && !entry["tag_id"]) {
      throw std::runtime_error("every tag prior entry must contain 'id' or 'tag_id'");
    }

    TagPrior prior;
    prior.tag_id = entry["id"] ? entry["id"].as<int>() : entry["tag_id"].as<int>();
    const YAML::Node pose_node = entry["pose"] ? entry["pose"] : entry;
    prior.world_T_tag = poseFromYamlNode(pose_node);
    const bool has_translation_sigma = entry["translation_sigma_m"] || entry["sigma_translation_m"];
    const bool has_rotation_sigma = entry["rotation_sigma_rad"] || entry["sigma_rotation_rad"];
    prior.translation_sigma_m = nodeDoubleOr(
      entry, "translation_sigma_m",
      nodeDoubleOr(entry, "sigma_translation_m", prior.translation_sigma_m));
    prior.rotation_sigma_rad = nodeDoubleOr(
      entry, "rotation_sigma_rad",
      nodeDoubleOr(entry, "sigma_rotation_rad", prior.rotation_sigma_rad));
    prior.has_custom_sigmas = has_translation_sigma || has_rotation_sigma;
    prior.anchor = nodeBoolOr(entry, "anchor", nodeBoolOr(entry, "is_anchor", false));
    prior.source_name = nodeStringOr(entry, "name", "tag_" + std::to_string(prior.tag_id));
    session.tag_priors.push_back(std::move(prior));
  }
}

void loadTagObservations(SessionData &session)
{
  for (const auto &keyframe : session.keyframes) {
    if (keyframe.tags_path.empty() || !std::filesystem::exists(keyframe.tags_path)) {
      continue;
    }

    const YAML::Node root = YAML::LoadFile(keyframe.tags_path.string());
    const YAML::Node detections = root["detections"];
    if (!detections || !detections.IsSequence()) {
      continue;
    }

    const std::string fallback_detection_frame = nodeStringOr(root, "header_frame_id", "");

    for (const auto &detection : detections) {
      const YAML::Node tf_pose = detection["tf_pose"];
      if (!tf_pose) {
        ++session.skipped_missing_tf_tag_observations;
        continue;
      }

      if (!nodeBoolOr(tf_pose, "available", false)) {
        ++session.skipped_unavailable_tag_observations;
        continue;
      }

      TagObservation observation;
      observation.kf_id = keyframe.kf_id;
      observation.tag_id = detection["id"].as<int>();
      observation.family = nodeStringOr(detection, "family", "");
      observation.parent_frame_id = nodeStringOr(tf_pose, "parent_frame_id", "");
      observation.child_frame_id = nodeStringOr(tf_pose, "child_frame_id", "");
      observation.detection_frame_id =
        nodeStringOr(tf_pose, "detection_frame_id", fallback_detection_frame);
      observation.lookup_stamp_ns = nodeInt64Or(tf_pose, "lookup_stamp_ns", 0);
      observation.source_path = keyframe.tags_path;

      if (observation.parent_frame_id != session.body_frame_id) {
        ++session.skipped_non_body_tag_observations;
        continue;
      }

      observation.body_T_tag = poseFromYamlNode(tf_pose);
      session.tag_observations.push_back(std::move(observation));
    }
  }
}

}  // namespace

SessionData loadSession(const std::filesystem::path &session_dir)
{
  SessionData session;
  session.session_dir = session_dir;
  session.session_name = session_dir.filename().string();

  if (!std::filesystem::exists(session.session_dir)) {
    throw std::runtime_error("session directory does not exist: " + session.session_dir.string());
  }

  loadSessionMetadata(session);
  loadManifest(session);
  loadKeyframeMetadata(session);
  loadTagPriors(session);
  loadTagObservations(session);

  if (session.keyframes.empty()) {
    throw std::runtime_error("session contains no keyframes: " + session.session_dir.string());
  }

  return session;
}

}  // namespace offline_global_graph
