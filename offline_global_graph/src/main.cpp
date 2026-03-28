#include "offline_global_graph/exporter.hpp"
#include "offline_global_graph/optimizer.hpp"
#include "offline_global_graph/session_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct CliConfig
{
  std::filesystem::path session_dir;
  std::filesystem::path output_dir;
  offline_global_graph::OptimizerConfig optimizer_config;
  std::vector<int> only_tag_ids;
  std::vector<int> exclude_tag_ids;
};

void printUsage()
{
  std::cerr
    << "Usage: offline_global_graph_cli --session-dir PATH [--output-dir PATH]\n"
    << "       [--only-tag-id INTEGER] [--exclude-tag-id INTEGER]\n"
    << "       [--between-translation-sigma FLOAT] [--between-rotation-sigma FLOAT]\n"
    << "       [--tag-translation-sigma FLOAT] [--tag-rotation-sigma FLOAT]\n"
    << "       [--soft-prior-translation-sigma FLOAT] [--soft-prior-rotation-sigma FLOAT]\n"
    << "       [--anchor-translation-sigma FLOAT] [--anchor-rotation-sigma FLOAT]\n"
    << "       [--anchor-all-tag-priors] [--no-anchor-all-tag-priors]\n"
    << "       [--max-tag-translation-deviation FLOAT] [--disable-max-tag-translation-deviation-check]\n"
    << "       [--tag-observation-huber-k FLOAT]\n"
    << "       [--body-to-camera-extrinsics PATH]\n"
    << "       [--visual-sigma-px FLOAT] [--visual-huber-k FLOAT]\n"
    << "       [--depth-scale FLOAT] [--min-depth FLOAT] [--max-depth FLOAT]\n"
    << "       [--min-track-observations INTEGER]\n"
    << "       [--robust-huber-k FLOAT]\n";
}

double parseCliDouble(const std::string &value, const char *flag)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse numeric value for ") + flag);
  }
}

int parseCliInt(const std::string &value, const char *flag)
{
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse integer value for ") + flag);
  }
}

std::string requireValue(int argc, char **argv, int &index, const char *flag)
{
  if ((index + 1) >= argc) {
    throw std::runtime_error(std::string("missing value for ") + flag);
  }
  ++index;
  return argv[index];
}

gtsam::Pose3 loadBodyToCameraExtrinsics(const std::filesystem::path &path)
{
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node pose_node = root["body_T_camera"] ? root["body_T_camera"] : root;
  const YAML::Node position = pose_node["position"] ? pose_node["position"] : pose_node["translation"];
  const YAML::Node quat = pose_node["orientation_xyzw"];
  if (!position || !quat || !position.IsSequence() || !quat.IsSequence() ||
      position.size() != 3 || quat.size() != 4) {
    throw std::runtime_error("extrinsics YAML must contain position/translation[3] and orientation_xyzw[4]");
  }

  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(
      quat[3].as<double>(),
      quat[0].as<double>(),
      quat[1].as<double>(),
      quat[2].as<double>()),
    gtsam::Point3(
      position[0].as<double>(),
      position[1].as<double>(),
      position[2].as<double>()));
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
  filtered.tag_priors.clear();

  for (const auto &observation : session.tag_observations) {
    if (tagAllowed(observation.tag_id, only_tag_ids, exclude_tag_ids)) {
      filtered.tag_observations.push_back(observation);
    }
  }

  for (const auto &prior : session.tag_priors) {
    if (tagAllowed(prior.tag_id, only_tag_ids, exclude_tag_ids)) {
      filtered.tag_priors.push_back(prior);
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
      } else if (arg == "--only-tag-id") {
        cli_config.only_tag_ids.push_back(
          parseCliInt(requireValue(argc, argv, i, "--only-tag-id"), "--only-tag-id"));
      } else if (arg == "--exclude-tag-id") {
        cli_config.exclude_tag_ids.push_back(
          parseCliInt(requireValue(argc, argv, i, "--exclude-tag-id"), "--exclude-tag-id"));
      } else if (arg == "--between-translation-sigma") {
        cli_config.optimizer_config.between_translation_sigma_m =
          parseCliDouble(requireValue(argc, argv, i, "--between-translation-sigma"), "--between-translation-sigma");
      } else if (arg == "--between-rotation-sigma") {
        cli_config.optimizer_config.between_rotation_sigma_rad =
          parseCliDouble(requireValue(argc, argv, i, "--between-rotation-sigma"), "--between-rotation-sigma");
      } else if (arg == "--tag-translation-sigma") {
        cli_config.optimizer_config.tag_translation_sigma_m =
          parseCliDouble(requireValue(argc, argv, i, "--tag-translation-sigma"), "--tag-translation-sigma");
      } else if (arg == "--tag-rotation-sigma") {
        cli_config.optimizer_config.tag_rotation_sigma_rad =
          parseCliDouble(requireValue(argc, argv, i, "--tag-rotation-sigma"), "--tag-rotation-sigma");
      } else if (arg == "--soft-prior-translation-sigma") {
        cli_config.optimizer_config.soft_prior_translation_sigma_m =
          parseCliDouble(requireValue(argc, argv, i, "--soft-prior-translation-sigma"), "--soft-prior-translation-sigma");
      } else if (arg == "--soft-prior-rotation-sigma") {
        cli_config.optimizer_config.soft_prior_rotation_sigma_rad =
          parseCliDouble(requireValue(argc, argv, i, "--soft-prior-rotation-sigma"), "--soft-prior-rotation-sigma");
      } else if (arg == "--anchor-translation-sigma") {
        cli_config.optimizer_config.anchor_translation_sigma_m =
          parseCliDouble(requireValue(argc, argv, i, "--anchor-translation-sigma"), "--anchor-translation-sigma");
      } else if (arg == "--anchor-rotation-sigma") {
        cli_config.optimizer_config.anchor_rotation_sigma_rad =
          parseCliDouble(requireValue(argc, argv, i, "--anchor-rotation-sigma"), "--anchor-rotation-sigma");
      } else if (arg == "--anchor-all-tag-priors") {
        cli_config.optimizer_config.anchor_all_tag_priors = true;
      } else if (arg == "--no-anchor-all-tag-priors") {
        cli_config.optimizer_config.anchor_all_tag_priors = false;
      } else if (arg == "--max-tag-translation-deviation") {
        cli_config.optimizer_config.max_tag_translation_deviation_m =
          parseCliDouble(
          requireValue(argc, argv, i, "--max-tag-translation-deviation"),
          "--max-tag-translation-deviation");
      } else if (arg == "--disable-max-tag-translation-deviation-check") {
        cli_config.optimizer_config.enforce_max_tag_translation_deviation = false;
      } else if (arg == "--tag-observation-huber-k") {
        cli_config.optimizer_config.tag_observation_huber_k =
          parseCliDouble(requireValue(argc, argv, i, "--tag-observation-huber-k"), "--tag-observation-huber-k");
      } else if (arg == "--body-to-camera-extrinsics") {
        cli_config.optimizer_config.body_T_camera =
          loadBodyToCameraExtrinsics(requireValue(argc, argv, i, "--body-to-camera-extrinsics"));
        cli_config.optimizer_config.use_visual_factors = true;
      } else if (arg == "--visual-sigma-px") {
        cli_config.optimizer_config.visual_sigma_px =
          parseCliDouble(requireValue(argc, argv, i, "--visual-sigma-px"), "--visual-sigma-px");
      } else if (arg == "--visual-huber-k") {
        cli_config.optimizer_config.visual_huber_k =
          parseCliDouble(requireValue(argc, argv, i, "--visual-huber-k"), "--visual-huber-k");
      } else if (arg == "--depth-scale") {
        cli_config.optimizer_config.depth_scale =
          parseCliDouble(requireValue(argc, argv, i, "--depth-scale"), "--depth-scale");
      } else if (arg == "--min-depth") {
        cli_config.optimizer_config.min_depth_m =
          parseCliDouble(requireValue(argc, argv, i, "--min-depth"), "--min-depth");
      } else if (arg == "--max-depth") {
        cli_config.optimizer_config.max_depth_m =
          parseCliDouble(requireValue(argc, argv, i, "--max-depth"), "--max-depth");
      } else if (arg == "--min-track-observations") {
        cli_config.optimizer_config.min_track_observations =
          parseCliInt(requireValue(argc, argv, i, "--min-track-observations"), "--min-track-observations");
      } else if (arg == "--robust-huber-k") {
        cli_config.optimizer_config.robust_huber_k =
          parseCliDouble(requireValue(argc, argv, i, "--robust-huber-k"), "--robust-huber-k");
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

    auto session = offline_global_graph::loadSession(cli_config.session_dir);
    session = filterSessionByTags(session, cli_config.only_tag_ids, cli_config.exclude_tag_ids);
    if (cli_config.output_dir.empty()) {
      cli_config.output_dir = cli_config.session_dir / "offline_global_graph";
    }

    const auto result = offline_global_graph::optimizeSession(session, cli_config.optimizer_config);
    offline_global_graph::writeOptimizationOutputs(session, result, cli_config.output_dir);

    std::cout
      << "Optimized session '" << session.session_name << "'\n"
      << "  keyframes: " << result.optimized_keyframes.size() << "\n"
      << "  tags: " << result.optimized_tags.size() << "\n"
      << "  tag_observations_used: " << session.tag_observations.size() << "\n"
      << "  visual_factors_used: " << result.visual_factor_count << "\n"
      << "  landmarks_used: " << result.landmark_count << "\n"
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
