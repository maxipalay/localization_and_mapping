#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace esdf_slice_map_publisher
{

struct GrayImage
{
  int width{0};
  int height{0};
  int max_value{255};
  std::vector<uint8_t> pixels;
};

struct MapSpec
{
  std::filesystem::path image_path;
  std::filesystem::path yaml_path;
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  int negate{0};
  double occupied_thresh{0.65};
  double free_thresh{0.196};
  std::string mode{"trinary"};
};

GrayImage loadPgm(const std::filesystem::path &path);
MapSpec loadMapSpec(const std::filesystem::path &yaml_path);
std::optional<double> parseSliceHeightFromPath(const std::filesystem::path &path);

}  // namespace esdf_slice_map_publisher
