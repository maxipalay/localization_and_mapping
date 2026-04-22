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
  offline_global_graph::OptimizerConfig optimizer_config;
};

void printUsage()
{
  std::cerr
    << "Usage: offline_global_graph_cli --session-dir PATH [--output-dir PATH]\n"
    << "       [--map-anchor-tag-priors PATH --map-anchor-tag-id INTEGER]\n";
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

void applyPosthocTagMapAlignment(
  offline_global_graph::OptimizationResult &result,
  const std::filesystem::path &tag_priors_path,
  int tag_id)
{
  const auto tag_it = std::find_if(
    result.optimized_tags.begin(),
    result.optimized_tags.end(),
    [tag_id](const auto &entry) { return entry.first == tag_id; });
  if (tag_it == result.optimized_tags.end()) {
    throw std::runtime_error(
            "cannot align optimized map: tag " + std::to_string(tag_id) +
            " is not present in optimized tag poses");
  }

  const gtsam::Pose3 map_T_tag = loadMapAnchorTagPose(tag_priors_path, tag_id);
  const gtsam::Pose3 map_T_session = map_T_tag * tag_it->second.inverse();

  for (auto &entry : result.optimized_keyframes) {
    entry.second = map_T_session * entry.second;
  }

  for (auto &entry : result.optimized_tags) {
    entry.second = map_T_session * entry.second;
  }

  result.has_posthoc_alignment = true;
  result.posthoc_alignment_tag_id = tag_id;
  result.posthoc_alignment_source_path = tag_priors_path.string();
  result.posthoc_alignment_map_T_session = map_T_session;
  if (!result.anchor_strategy.empty()) {
    result.anchor_strategy += " + ";
  }
  result.anchor_strategy += "posthoc_tag_map_align:" + std::to_string(tag_id);
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
    if (cli_config.output_dir.empty()) {
      cli_config.output_dir = cli_config.session_dir / "offline_global_graph";
    }

    auto result = offline_global_graph::optimizeSession(session, cli_config.optimizer_config);
    if (cli_config.map_anchor_tag_id) {
      applyPosthocTagMapAlignment(
        result,
        cli_config.map_anchor_tag_priors_path,
        *cli_config.map_anchor_tag_id);
    }
    offline_global_graph::writeOptimizationOutputs(session, result, cli_config.output_dir);

    std::cout
      << "Optimized session '" << session.session_name << "'\n"
      << "  keyframes: " << result.optimized_keyframes.size() << "\n"
      << "  tags: " << result.optimized_tags.size() << "\n"
      << "  between_factors: " << result.between_factor_count << "\n"
      << "  between_factors_inflated: " << result.between_factor_inflated_count << "\n"
      << "  between_factors_skipped: " << result.between_factor_skipped_count << "\n"
      << "  tag_observations_used: " << session.tag_observations.size() << "\n"
      << "  optimizer_pose_priors_used: " << result.optimizer_pose_prior_count << "\n"
      << "  visual_factors_used: " << result.visual_factor_count << "\n"
      << "  landmarks_used: " << result.landmark_count << "\n"
      << "  initial_error: " << result.initial_error << "\n"
      << "  final_error: " << result.final_error << "\n"
      << "  anchor_strategy: " << result.anchor_strategy << "\n"
      << "  output_dir: " << cli_config.output_dir.string() << "\n";
    if (result.has_posthoc_alignment) {
      std::cout
        << "  posthoc_alignment_tag_id: " << result.posthoc_alignment_tag_id << "\n"
        << "  posthoc_alignment_source: " << result.posthoc_alignment_source_path << "\n";
    }

    return 0;
  } catch (const std::exception &ex) {
    std::cerr << "offline_global_graph_cli: " << ex.what() << "\n";
    return 1;
  }
}
