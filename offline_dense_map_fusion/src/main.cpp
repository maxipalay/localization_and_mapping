#include "offline_dense_map_fusion/exporter.hpp"
#include "offline_dense_map_fusion/fuser.hpp"
#include "offline_dense_map_fusion/session_loader.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void printUsage()
{
  std::cerr
    << "Usage: offline_dense_map_fusion_cli --session-dir PATH --body-to-camera-extrinsics PATH\n"
    << "       [--output-dir PATH] [--voxel-size FLOAT] [--depth-scale FLOAT]\n"
    << "       [--min-depth FLOAT] [--max-depth FLOAT] [--pixel-stride INT]\n"
    << "       [--crop-border-px INT] [--max-world-z FLOAT]\n"
    << "       [--truncation-distance-vox FLOAT] [--max-weight FLOAT]\n"
    << "       [--mesh-min-weight FLOAT]\n";
}

std::string requireValue(int argc, char **argv, int &index, const char *flag)
{
  if ((index + 1) >= argc) {
    throw std::runtime_error(std::string("missing value for ") + flag);
  }
  ++index;
  return argv[index];
}

double parseDouble(const std::string &value, const char *flag)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse numeric value for ") + flag);
  }
}

int parseInt(const std::string &value, const char *flag)
{
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse integer value for ") + flag);
  }
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    std::filesystem::path session_dir;
    std::filesystem::path output_dir;
    std::filesystem::path extrinsics_path;
    offline_dense_map_fusion::FusionConfig config;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--session-dir") {
        session_dir = requireValue(argc, argv, i, "--session-dir");
      } else if (arg == "--output-dir") {
        output_dir = requireValue(argc, argv, i, "--output-dir");
      } else if (arg == "--body-to-camera-extrinsics") {
        extrinsics_path = requireValue(argc, argv, i, "--body-to-camera-extrinsics");
      } else if (arg == "--voxel-size") {
        config.voxel_size_m = parseDouble(requireValue(argc, argv, i, "--voxel-size"), "--voxel-size");
      } else if (arg == "--depth-scale") {
        config.depth_scale = parseDouble(requireValue(argc, argv, i, "--depth-scale"), "--depth-scale");
      } else if (arg == "--min-depth") {
        config.min_depth_m = parseDouble(requireValue(argc, argv, i, "--min-depth"), "--min-depth");
      } else if (arg == "--max-depth") {
        config.max_depth_m = parseDouble(requireValue(argc, argv, i, "--max-depth"), "--max-depth");
      } else if (arg == "--max-world-z") {
        config.max_world_z_m = parseDouble(requireValue(argc, argv, i, "--max-world-z"), "--max-world-z");
      } else if (arg == "--pixel-stride") {
        config.pixel_stride = parseInt(requireValue(argc, argv, i, "--pixel-stride"), "--pixel-stride");
      } else if (arg == "--crop-border-px") {
        config.crop_border_px = parseInt(requireValue(argc, argv, i, "--crop-border-px"), "--crop-border-px");
      } else if (arg == "--truncation-distance-vox") {
        config.truncation_distance_vox =
          parseDouble(requireValue(argc, argv, i, "--truncation-distance-vox"), "--truncation-distance-vox");
      } else if (arg == "--max-weight") {
        config.max_weight =
          parseDouble(requireValue(argc, argv, i, "--max-weight"), "--max-weight");
      } else if (arg == "--mesh-min-weight") {
        config.mesh_min_weight =
          parseDouble(requireValue(argc, argv, i, "--mesh-min-weight"), "--mesh-min-weight");
      } else if (arg == "--help" || arg == "-h") {
        printUsage();
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }

    if (session_dir.empty() || extrinsics_path.empty()) {
      printUsage();
      return 1;
    }

    const auto session = offline_dense_map_fusion::loadSessionWithOptimizedPoses(session_dir);
    const auto extrinsics = offline_dense_map_fusion::loadExtrinsics(extrinsics_path);
    if (output_dir.empty()) {
      output_dir = session_dir / "offline_dense_map_fusion";
    }

    const auto result = offline_dense_map_fusion::fuseSession(session, extrinsics, config, output_dir);
    offline_dense_map_fusion::writeFusionOutputs(session, extrinsics, config, result, output_dir);

    std::cout
      << "Fused dense map for session '" << session.session_name << "'\n"
      << "  frames: " << result.frames_fused << "\n"
      << "  raw_points_considered: " << result.raw_points_considered << "\n"
      << "  mesh_path: " << result.mesh_path.string() << "\n"
      << "  mesh_blocks: " << result.mesh_block_count << "\n"
      << "  output_dir: " << output_dir.string() << "\n";
    return 0;
  } catch (const std::exception &ex) {
    std::cerr << "offline_dense_map_fusion_cli: " << ex.what() << "\n";
    return 1;
  }
}
