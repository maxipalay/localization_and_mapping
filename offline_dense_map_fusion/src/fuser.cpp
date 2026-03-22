#include "offline_dense_map_fusion/fuser.hpp"

#include <nvblox/core/color.h>
#include <nvblox/core/types.h>
#include <nvblox/io/mesh_io.h>
#include <nvblox/mapper/mapper.h>
#include <nvblox/mapper/mapper_params.h>
#include <nvblox/sensors/camera.h>
#include <nvblox/sensors/image.h>

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace offline_dense_map_fusion
{

namespace
{

Pose compose(const Pose &lhs, const Pose &rhs)
{
  Pose out;
  out.rotation = lhs.rotation * rhs.rotation;
  out.translation = lhs.rotation * rhs.translation + lhs.translation;
  return out;
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

nvblox::Transform toNvbloxTransform(const Pose &pose)
{
  nvblox::Transform T = nvblox::Transform::Identity();
  T.linear() << static_cast<float>(pose.rotation(0, 0)), static_cast<float>(pose.rotation(0, 1)), static_cast<float>(pose.rotation(0, 2)),
    static_cast<float>(pose.rotation(1, 0)), static_cast<float>(pose.rotation(1, 1)), static_cast<float>(pose.rotation(1, 2)),
    static_cast<float>(pose.rotation(2, 0)), static_cast<float>(pose.rotation(2, 1)), static_cast<float>(pose.rotation(2, 2));
  T.translation() << static_cast<float>(pose.translation[0]),
    static_cast<float>(pose.translation[1]),
    static_cast<float>(pose.translation[2]);
  return T;
}

}  // namespace

FusionResult fuseSession(
  const SessionData &session,
  const Extrinsics &extrinsics,
  const FusionConfig &config,
  const std::filesystem::path &output_dir)
{
  if (config.voxel_size_m <= 0.0) {
    throw std::runtime_error("voxel_size_m must be positive");
  }
  if (config.pixel_stride <= 0) {
    throw std::runtime_error("pixel_stride must be positive");
  }

  FusionResult result;

  nvblox::Mapper mapper(
    static_cast<float>(config.voxel_size_m),
    nvblox::BlockMemoryPoolParams(),
    nvblox::ProjectiveLayerType::kTsdf);

  nvblox::MapperParams mapper_params;
  mapper_params.projective_integrator_params.projective_integrator_max_integration_distance_m =
    static_cast<float>(config.max_depth_m);
  mapper.setMapperParams(mapper_params);

  const int stride = std::max(1, config.pixel_stride);
  const int camera_width = (session.camera.width + stride - 1) / stride;
  const int camera_height = (session.camera.height + stride - 1) / stride;
  nvblox::Camera camera(
    static_cast<float>(session.camera.fx / stride),
    static_cast<float>(session.camera.fy / stride),
    static_cast<float>(session.camera.cx / stride),
    static_cast<float>(session.camera.cy / stride),
    camera_width,
    camera_height);

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
    const int sampled_rows = (rows + stride - 1) / stride;
    const int sampled_cols = (cols + stride - 1) / stride;

    std::vector<float> depth_buffer(static_cast<size_t>(sampled_rows * sampled_cols), 0.0f);
    std::vector<nvblox::Color> color_buffer(static_cast<size_t>(sampled_rows * sampled_cols));

    for (int out_y = 0; out_y < sampled_rows; ++out_y) {
      const int src_y = std::min(out_y * stride, rows - 1);
      for (int out_x = 0; out_x < sampled_cols; ++out_x) {
        const int src_x = std::min(out_x * stride, cols - 1);
        const size_t linear_idx = static_cast<size_t>(out_y * sampled_cols + out_x);

        const double depth_m = depthMetersAt(depth, src_y, src_x, config.depth_scale);
        if (std::isfinite(depth_m) && depth_m >= config.min_depth_m && depth_m <= config.max_depth_m) {
          depth_buffer[linear_idx] = static_cast<float>(depth_m);
          ++result.raw_points_considered;
        }

        const cv::Vec3b bgr = colorAt(rgb, src_y, src_x);
        color_buffer[linear_idx] = nvblox::Color(bgr[2], bgr[1], bgr[0]);
      }
    }

    nvblox::DepthImage depth_image(nvblox::MemoryType::kUnified);
    depth_image.copyFrom(
      static_cast<size_t>(sampled_rows),
      static_cast<size_t>(sampled_cols),
      depth_buffer.data());

    nvblox::ColorImage color_image(nvblox::MemoryType::kUnified);
    color_image.copyFrom(
      static_cast<size_t>(sampled_rows),
      static_cast<size_t>(sampled_cols),
      color_buffer.data());

    const nvblox::Transform T_L_C = toNvbloxTransform(world_T_camera);
    mapper.integrateDepth(depth_image, T_L_C, camera);
    mapper.integrateColor(color_image, T_L_C, camera);
    ++result.frames_fused;
  }

  mapper.updateColorMesh(nvblox::UpdateFullLayer::kYes);

  std::filesystem::create_directories(output_dir);
  result.mesh_path = output_dir / "fused_mesh.ply";
  if (!nvblox::io::outputColorMeshLayerToPly(
        mapper.color_mesh_layer(),
        result.mesh_path.string())) {
    throw std::runtime_error("nvblox failed to write mesh PLY: " + result.mesh_path.string());
  }
  result.mesh_block_count = mapper.color_mesh_layer().size();

  return result;
}

}  // namespace offline_dense_map_fusion
