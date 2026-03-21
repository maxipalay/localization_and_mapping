#include "offline_dense_map_fusion/fuser.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace offline_dense_map_fusion
{

namespace
{

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey &key) const
  {
    size_t h = 1469598103934665603ULL;
    h ^= static_cast<size_t>(key.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

Pose compose(const Pose &lhs, const Pose &rhs)
{
  Pose out;
  out.rotation = lhs.rotation * rhs.rotation;
  out.translation = lhs.rotation * rhs.translation + lhs.translation;
  return out;
}

cv::Vec3f transformPoint(const Pose &pose, const cv::Vec3f &point)
{
  const cv::Vec3d p(point[0], point[1], point[2]);
  const cv::Vec3d transformed = pose.rotation * p + pose.translation;
  return cv::Vec3f(
    static_cast<float>(transformed[0]),
    static_cast<float>(transformed[1]),
    static_cast<float>(transformed[2]));
}

double depthMetersAt(const cv::Mat &depth, int y, int x, double depth_scale)
{
  switch (depth.type()) {
    case CV_16UC1:
      return static_cast<double>(depth.at<uint16_t>(y, x)) * depth_scale;
    case CV_32FC1:
      return static_cast<double>(depth.at<float>(y, x)) * depth_scale;
    case CV_64FC1:
      return depth.at<double>(y, x) * depth_scale;
    default:
      throw std::runtime_error("unsupported depth image type");
  }
}

cv::Vec3b colorAt(const cv::Mat &rgb, int y, int x)
{
  if (rgb.type() == CV_8UC3) {
    return rgb.at<cv::Vec3b>(y, x);
  }
  if (rgb.type() == CV_8UC1) {
    const uint8_t v = rgb.at<uint8_t>(y, x);
    return cv::Vec3b(v, v, v);
  }
  throw std::runtime_error("unsupported RGB image type");
}

}  // namespace

FusionResult fuseSession(
  const SessionData &session,
  const Extrinsics &extrinsics,
  const FusionConfig &config)
{
  FusionResult result;
  std::unordered_map<VoxelKey, ColoredPoint, VoxelKeyHash> voxels;

  for (const auto &frame : session.frames) {
    const cv::Mat rgb = cv::imread(frame.rgb_path.string(), cv::IMREAD_UNCHANGED);
    const cv::Mat depth = cv::imread(frame.depth_path.string(), cv::IMREAD_UNCHANGED);
    if (rgb.empty()) {
      throw std::runtime_error("failed to read RGB image: " + frame.rgb_path.string());
    }
    if (depth.empty()) {
      throw std::runtime_error("failed to read depth image: " + frame.depth_path.string());
    }

    const Pose world_T_camera = compose(frame.world_T_body, extrinsics.body_T_camera);
    result.world_T_camera_by_kf.emplace_back(frame.kf_id, world_T_camera);

    const int rows = std::min(depth.rows, rgb.rows);
    const int cols = std::min(depth.cols, rgb.cols);
    for (int y = 0; y < rows; y += std::max(1, config.pixel_stride)) {
      for (int x = 0; x < cols; x += std::max(1, config.pixel_stride)) {
        const double depth_m = depthMetersAt(depth, y, x, config.depth_scale);
        if (!std::isfinite(depth_m) || depth_m < config.min_depth_m || depth_m > config.max_depth_m) {
          continue;
        }

        ++result.raw_points_considered;

        const float z = static_cast<float>(depth_m);
        const float px = static_cast<float>((static_cast<double>(x) - session.camera.cx) * depth_m / session.camera.fx);
        const float py = static_cast<float>((static_cast<double>(y) - session.camera.cy) * depth_m / session.camera.fy);
        const cv::Vec3f point_camera(px, py, z);
        const cv::Vec3f point_world = transformPoint(world_T_camera, point_camera);

        const VoxelKey key{
          static_cast<int>(std::floor(point_world[0] / config.voxel_size_m)),
          static_cast<int>(std::floor(point_world[1] / config.voxel_size_m)),
          static_cast<int>(std::floor(point_world[2] / config.voxel_size_m))
        };

        voxels.insert_or_assign(key, ColoredPoint{point_world, colorAt(rgb, y, x)});
      }
    }

    ++result.frames_fused;
  }

  result.points.reserve(voxels.size());
  for (const auto &entry : voxels) {
    result.points.push_back(entry.second);
  }

  return result;
}

}  // namespace offline_dense_map_fusion
