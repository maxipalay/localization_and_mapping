#include "offline_global_graph/exporter.hpp"
#include "offline_global_graph/optimizer.hpp"
#include "offline_global_graph/session_loader.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

struct CliConfig
{
  std::filesystem::path session_dir;
  std::filesystem::path output_dir;
  offline_global_graph::OptimizerConfig optimizer_config;
};

void printUsage()
{
  std::cerr
    << "Usage: offline_global_graph_cli --session-dir PATH [--output-dir PATH]\n";
}

std::string requireValue(int argc, char **argv, int &index, const char *flag)
{
  if ((index + 1) >= argc) {
    throw std::runtime_error(std::string("missing value for ") + flag);
  }
  ++index;
  return argv[index];
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
    if (cli_config.output_dir.empty()) {
      cli_config.output_dir = cli_config.session_dir / "offline_global_graph";
    }

    const auto result = offline_global_graph::optimizeSession(session, cli_config.optimizer_config);
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

    return 0;
  } catch (const std::exception &ex) {
    std::cerr << "offline_global_graph_cli: " << ex.what() << "\n";
    return 1;
  }
}
