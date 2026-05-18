#include "offline_global_graph/exporter.hpp"
#include "offline_global_graph/optimizer.hpp"
#include "offline_global_graph/session_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <optional>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

struct CliConfig
{
  std::filesystem::path session_dir;
  std::filesystem::path output_dir;
  std::filesystem::path map_anchor_tag_priors_path;
  std::optional<int> map_anchor_tag_id;
  std::vector<int> only_tag_ids;
  std::vector<int> exclude_tag_ids;
  offline_global_graph::OptimizerConfig optimizer_config;
};

void printUsage()
{
  std::cerr
    << "Usage: offline_global_graph_cli --session-dir PATH [--output-dir PATH]\n"
    << "       [--map-anchor-tag-priors PATH --map-anchor-tag-id INTEGER]\n"
    << "       [--only-tag-id INTEGER] [--exclude-tag-id INTEGER]\n";
}

std::string requireValue(int argc, char **argv, int &index, const char *flag)
{
  if ((index + 1) >= argc) {
    throw std::runtime_error(std::string("missing value for ") + flag);
  }
  ++index;
  return argv[index];
}

int parseInt(const std::string &value, const char *flag)
{
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse integer value for ") + flag);
  }
}

gtsam::Pose3 poseFromYamlNode(const YAML::Node &node)
{
  const YAML::Node translation = node["position"] ? node["position"] : node["translation"];
  const YAML::Node quaternion = node["orientation_xyzw"];
  if (!translation || !quaternion || !translation.IsSequence() || !quaternion.IsSequence() ||
      translation.size() != 3 || quaternion.size() != 4) {
    throw std::runtime_error("tag pose must contain position/translation[3] and orientation_xyzw[4]");
  }

  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(
      quaternion[3].as<double>(),
      quaternion[0].as<double>(),
      quaternion[1].as<double>(),
      quaternion[2].as<double>()),
    gtsam::Point3(
      translation[0].as<double>(),
      translation[1].as<double>(),
      translation[2].as<double>()));
}

gtsam::Pose3 loadMapAnchorTagPose(
  const std::filesystem::path &tag_priors_path,
  int tag_id)
{
  const YAML::Node root = YAML::LoadFile(tag_priors_path.string());
  const YAML::Node tags = root["tags"];
  if (!tags || !tags.IsSequence()) {
    throw std::runtime_error("tag priors file is missing a 'tags' sequence: " + tag_priors_path.string());
  }

  for (const auto &entry : tags) {
    if (!entry["id"]) {
      continue;
    }
    if (entry["id"].as<int>() == tag_id) {
      return poseFromYamlNode(entry);
    }
  }

  throw std::runtime_error(
          "tag priors file does not contain requested tag id " + std::to_string(tag_id) + ": " +
          tag_priors_path.string());
}

bool tagAllowed(
  int tag_id,
  const std::vector<int> &only_tag_ids,
  const std::vector<int> &exclude_tag_ids)
{
  if (!only_tag_ids.empty() &&
      std::find(only_tag_ids.begin(), only_tag_ids.end(), tag_id) == only_tag_ids.end()) {
    return false;
  }

  return std::find(exclude_tag_ids.begin(), exclude_tag_ids.end(), tag_id) == exclude_tag_ids.end();
}

offline_global_graph::SessionData filterSessionByTags(
  const offline_global_graph::SessionData &session,
  const std::vector<int> &only_tag_ids,
  const std::vector<int> &exclude_tag_ids)
{
  if (only_tag_ids.empty() && exclude_tag_ids.empty()) {
    return session;
  }

  offline_global_graph::SessionData filtered = session;
  filtered.tag_observations.clear();

  for (const auto &observation : session.tag_observations) {
    if (tagAllowed(observation.tag_id, only_tag_ids, exclude_tag_ids)) {
      filtered.tag_observations.push_back(observation);
    }
  }

  return filtered;
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    CliConfig cli_config;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--session-dir") {
        cli_config.session_dir = requireValue(argc, argv, i, "--session-dir");
      } else if (arg == "--output-dir") {
        cli_config.output_dir = requireValue(argc, argv, i, "--output-dir");
      } else if (arg == "--map-anchor-tag-priors") {
        cli_config.map_anchor_tag_priors_path = requireValue(argc, argv, i, "--map-anchor-tag-priors");
      } else if (arg == "--map-anchor-tag-id") {
        cli_config.map_anchor_tag_id = parseInt(
          requireValue(argc, argv, i, "--map-anchor-tag-id"), "--map-anchor-tag-id");
      } else if (arg == "--only-tag-id") {
        cli_config.only_tag_ids.push_back(
          parseInt(requireValue(argc, argv, i, "--only-tag-id"), "--only-tag-id"));
      } else if (arg == "--exclude-tag-id") {
        cli_config.exclude_tag_ids.push_back(
          parseInt(requireValue(argc, argv, i, "--exclude-tag-id"), "--exclude-tag-id"));
      } else if (arg == "--help" || arg == "-h") {
        printUsage();
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }

    if (cli_config.session_dir.empty()) {
      printUsage();
      return 1;
    }
    if (cli_config.map_anchor_tag_priors_path.empty() != !cli_config.map_anchor_tag_id.has_value()) {
      throw std::runtime_error(
        "--map-anchor-tag-priors and --map-anchor-tag-id must be provided together");
    }

    auto session = offline_global_graph::loadSession(cli_config.session_dir);
    session = filterSessionByTags(session, cli_config.only_tag_ids, cli_config.exclude_tag_ids);
    if (cli_config.output_dir.empty()) {
      cli_config.output_dir = cli_config.session_dir / "offline_global_graph";
    }
    if (cli_config.map_anchor_tag_id) {
      cli_config.optimizer_config.use_tag_anchor_prior = true;
      cli_config.optimizer_config.tag_anchor_id = *cli_config.map_anchor_tag_id;
      cli_config.optimizer_config.tag_anchor_pose = loadMapAnchorTagPose(
        cli_config.map_anchor_tag_priors_path,
        *cli_config.map_anchor_tag_id);
    }

    auto result = offline_global_graph::optimizeSession(session, cli_config.optimizer_config);
    offline_global_graph::writeOptimizationOutputs(session, result, cli_config.output_dir);

    std::cout
      << "Optimized session '" << session.session_name << "'\n"
      << "  keyframes: " << result.optimized_keyframes.size() << "\n"
      << "  tags: " << result.optimized_tags.size() << "\n"
      << "  filtered_tag_observations_input: " << session.tag_observations.size() << "\n"
      << "  between_factors: " << result.between_factor_count << "\n"
      << "  tag_observations_used: " << result.tag_observation_factor_count << "\n"
      << "  tag_observations_skipped_distance: " << result.tag_observation_skipped_distance_count << "\n"
      << "  tag_observations_skipped_hamming: " << result.tag_observation_skipped_hamming_count << "\n"
      << "  tag_observations_skipped_low_margin: " << result.tag_observation_skipped_low_margin_count << "\n"
      << "  tag_observations_skipped_oblique: " << result.tag_observation_skipped_oblique_count << "\n"
      << "  initial_error: " << result.initial_error << "\n"
      << "  final_error: " << result.final_error << "\n"
      << "  anchor_strategy: " << result.anchor_strategy << "\n"
      << "  output_dir: " << cli_config.output_dir.string() << "\n";

    return 0;
  } catch (const std::exception &ex) {
    std::cerr << "offline_global_graph_cli: " << ex.what() << "\n";
    return 1;
  }
}
