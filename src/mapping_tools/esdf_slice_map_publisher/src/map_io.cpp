#include "map_io.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>

namespace esdf_slice_map_publisher
{

namespace
{

std::string readNextPgmToken(std::istream &stream)
{
  while (true) {
    const int next = stream.peek();
    if (next == EOF) {
      throw std::runtime_error("unexpected EOF while reading PGM header");
    }
    if (std::isspace(next)) {
      stream.get();
      continue;
    }
    if (next == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    break;
  }

  std::string token;
  while (true) {
    const int next = stream.peek();
    if (next == EOF || std::isspace(next) || next == '#') {
      break;
    }
    token.push_back(static_cast<char>(stream.get()));
  }

  if (token.empty()) {
    throw std::runtime_error("failed to read token from PGM");
  }
  return token;
}

void skipPgmWhitespaceAndComments(std::istream &stream)
{
  while (true) {
    const int next = stream.peek();
    if (next == EOF) {
      return;
    }
    if (std::isspace(next)) {
      stream.get();
      continue;
    }
    if (next == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    return;
  }
}

}  // namespace

GrayImage loadPgm(const std::filesystem::path &path)
{
  std::ifstream stream(path, std::ios::in | std::ios::binary);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open image: " + path.string());
  }

  const std::string magic = readNextPgmToken(stream);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("unsupported PGM format '" + magic + "' in " + path.string());
  }

  GrayImage image;
  image.width = std::stoi(readNextPgmToken(stream));
  image.height = std::stoi(readNextPgmToken(stream));
  image.max_value = std::stoi(readNextPgmToken(stream));
  if (image.width <= 0 || image.height <= 0) {
    throw std::runtime_error("invalid PGM dimensions in " + path.string());
  }
  if (image.max_value <= 0 || image.max_value > 255) {
    throw std::runtime_error("unsupported PGM max value in " + path.string());
  }

  const size_t pixel_count = static_cast<size_t>(image.width) * static_cast<size_t>(image.height);
  image.pixels.resize(pixel_count);

  if (magic == "P5") {
    skipPgmWhitespaceAndComments(stream);
    stream.read(reinterpret_cast<char *>(image.pixels.data()), static_cast<std::streamsize>(pixel_count));
    if (stream.gcount() != static_cast<std::streamsize>(pixel_count)) {
      throw std::runtime_error("unexpected EOF while reading PGM raster from " + path.string());
    }
  } else {
    for (size_t i = 0; i < pixel_count; ++i) {
      const int value = std::stoi(readNextPgmToken(stream));
      image.pixels[i] = static_cast<uint8_t>(std::clamp(value, 0, image.max_value));
    }
  }

  return image;
}

MapSpec loadMapSpec(const std::filesystem::path &yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path.string());
  MapSpec spec;
  spec.yaml_path = std::filesystem::weakly_canonical(yaml_path);

  const auto image_field = root["image"];
  const auto resolution_field = root["resolution"];
  const auto origin_field = root["origin"];
  if (!image_field || !resolution_field || !origin_field || !origin_field.IsSequence() ||
      origin_field.size() < 3)
  {
    throw std::runtime_error("map YAML is missing required fields");
  }

  std::filesystem::path image_path = image_field.as<std::string>();
  if (image_path.is_relative()) {
    image_path = yaml_path.parent_path() / image_path;
  }

  spec.image_path = std::filesystem::weakly_canonical(image_path);
  spec.resolution = resolution_field.as<double>();
  spec.origin_x = origin_field[0].as<double>();
  spec.origin_y = origin_field[1].as<double>();
  spec.origin_yaw = origin_field[2].as<double>();
  if (root["negate"]) {
    spec.negate = root["negate"].as<int>();
  }
  if (root["occupied_thresh"]) {
    spec.occupied_thresh = root["occupied_thresh"].as<double>();
  }
  if (root["free_thresh"]) {
    spec.free_thresh = root["free_thresh"].as<double>();
  }
  if (root["mode"]) {
    spec.mode = root["mode"].as<std::string>();
  }

  if (spec.resolution <= 0.0) {
    throw std::runtime_error("map resolution must be positive");
  }

  return spec;
}

std::optional<double> parseSliceHeightFromPath(const std::filesystem::path &path)
{
  const std::string stem = path.stem().string();
  const std::string needle = "esdf_slice_z_";
  const size_t start = stem.find(needle);
  if (start == std::string::npos) {
    return std::nullopt;
  }

  const size_t value_start = start + needle.size();
  const size_t value_end = stem.find("_occupancy", value_start);
  const std::string encoded = stem.substr(value_start, value_end - value_start);
  if (encoded.empty()) {
    return std::nullopt;
  }

  for (size_t dot_idx = encoded.find('_'); dot_idx != std::string::npos;
       dot_idx = encoded.find('_', dot_idx + 1))
  {
    std::string candidate = encoded;
    candidate[dot_idx] = '.';
    try {
      return std::stod(candidate);
    } catch (const std::exception &) {
    }
  }

  try {
    return std::stod(encoded);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace esdf_slice_map_publisher
