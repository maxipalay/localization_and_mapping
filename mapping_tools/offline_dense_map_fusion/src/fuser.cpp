#include "offline_dense_map_fusion/fuser.hpp"

#include <nvblox/core/color.h>
#include <nvblox/core/indexing.h>
#include <nvblox/core/types.h>
#include <nvblox/integrators/weighting_function.h>
#include <nvblox/io/mesh_io.h>
#include <nvblox/io/pointcloud_io.h>
#include <nvblox/map/accessors.h>
#include <nvblox/mapper/mapper.h>
#include <nvblox/mapper/mapper_params.h>
#include <nvblox/sensors/camera.h>
#include <nvblox/sensors/image.h>

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
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

cv::Vec3d transformPoint(const Pose &pose, const cv::Vec3d &point)
{
  return pose.rotation * point + pose.translation;
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

std::optional<nvblox::RadialTangentialDistortionParams> toNvbloxDistortion(
  const CameraIntrinsics &camera)
{
  if (camera.distortion_coeffs.empty()) {
    return std::nullopt;
  }

  const std::string &model = camera.distortion_model;
  if (!model.empty() && model != "plumb_bob" && model != "rational_polynomial") {
    return std::nullopt;
  }

  nvblox::RadialTangentialDistortionParams distortion;
  if (camera.distortion_coeffs.size() > 0) {
    distortion.radial.k1 = static_cast<float>(camera.distortion_coeffs[0]);
  }
  if (camera.distortion_coeffs.size() > 1) {
    distortion.radial.k2 = static_cast<float>(camera.distortion_coeffs[1]);
  }
  if (camera.distortion_coeffs.size() > 2) {
    distortion.tangential.p1 = static_cast<float>(camera.distortion_coeffs[2]);
  }
  if (camera.distortion_coeffs.size() > 3) {
    distortion.tangential.p2 = static_cast<float>(camera.distortion_coeffs[3]);
  }
  if (camera.distortion_coeffs.size() > 4) {
    distortion.radial.k3 = static_cast<float>(camera.distortion_coeffs[4]);
  }
  if (camera.distortion_coeffs.size() > 5) {
    distortion.radial.k4 = static_cast<float>(camera.distortion_coeffs[5]);
  }
  if (camera.distortion_coeffs.size() > 6) {
    distortion.radial.k5 = static_cast<float>(camera.distortion_coeffs[6]);
  }
  if (camera.distortion_coeffs.size() > 7) {
    distortion.radial.k6 = static_cast<float>(camera.distortion_coeffs[7]);
  }

  const bool all_zero =
    std::abs(distortion.radial.k1) < 1e-12f &&
    std::abs(distortion.radial.k2) < 1e-12f &&
    std::abs(distortion.radial.k3) < 1e-12f &&
    std::abs(distortion.radial.k4) < 1e-12f &&
    std::abs(distortion.radial.k5) < 1e-12f &&
    std::abs(distortion.radial.k6) < 1e-12f &&
    std::abs(distortion.tangential.p1) < 1e-12f &&
    std::abs(distortion.tangential.p2) < 1e-12f;
  if (all_zero) {
    return std::nullopt;
  }
  return distortion;
}

std::string sanitizeForFilename(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  std::string sanitized = stream.str();
  for (char &c : sanitized) {
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
      continue;
    }
    c = '_';
  }
  return sanitized;
}

void exportEsdfSliceOccupancyMap(
  const nvblox::EsdfLayer &layer,
  double slice_height_m,
  const std::filesystem::path &output_dir,
  FusionResult *result)
{
  struct Bounds
  {
    bool has_value{false};
    float min_x{0.0f};
    float max_x{0.0f};
    float min_y{0.0f};
    float max_y{0.0f};
  };

  Bounds bounds;
  const float block_size_m = layer.block_size();
  const float voxel_size_m = layer.voxel_size();
  const double band_half_thickness_m = static_cast<double>(voxel_size_m) * 0.5 + 1e-6;

  nvblox::callFunctionOnAllVoxels<nvblox::EsdfVoxel>(
    layer,
    nvblox::ConstVoxelCallbackFunction<nvblox::EsdfVoxel>(
      [&](const nvblox::Index3D &block_index,
          const nvblox::Index3D &voxel_index,
          const nvblox::EsdfVoxel *voxel) {
        if (voxel == nullptr || !voxel->observed) {
          return;
        }
        const nvblox::Vector3f center =
          nvblox::getCenterPositionFromBlockIndexAndVoxelIndex(
          block_size_m, block_index, voxel_index);
        if (std::abs(static_cast<double>(center.z()) - slice_height_m) > band_half_thickness_m) {
          return;
        }

        if (!bounds.has_value) {
          bounds.has_value = true;
          bounds.min_x = center.x();
          bounds.max_x = center.x();
          bounds.min_y = center.y();
          bounds.max_y = center.y();
          return;
        }

        bounds.min_x = std::min(bounds.min_x, center.x());
        bounds.max_x = std::max(bounds.max_x, center.x());
        bounds.min_y = std::min(bounds.min_y, center.y());
        bounds.max_y = std::max(bounds.max_y, center.y());
      }));

  using HostBlockPtr = nvblox::VoxelBlock<nvblox::EsdfVoxel>::Ptr;
  nvblox::Index3DHashMapType<HostBlockPtr>::type host_block_cache;
  const auto getVoxelForSampling =
    [&](const nvblox::Vector3f &sample_position, const nvblox::EsdfVoxel **voxel) -> bool {
      if (layer.memory_type() != nvblox::MemoryType::kDevice) {
        return nvblox::getVoxelAtPosition(layer, sample_position, voxel);
      }

      nvblox::Index3D block_index;
      nvblox::Index3D voxel_index;
      nvblox::getBlockAndVoxelIndexFromPositionInLayer(
        block_size_m, sample_position, &block_index, &voxel_index);
      auto it = host_block_cache.find(block_index);
      if (it == host_block_cache.end()) {
        const auto device_block = layer.getBlockAtIndex(block_index);
        if (!device_block) {
          return false;
        }
        it = host_block_cache.emplace(block_index, device_block.clone(nvblox::MemoryType::kHost)).first;
      }

      *voxel = &it->second->voxels[voxel_index.x()][voxel_index.y()][voxel_index.z()];
      return true;
    };

  if (!bounds.has_value) {
    throw std::runtime_error(
      "no observed ESDF voxels intersect requested slice height " +
      std::to_string(slice_height_m));
  }

  const auto cellsAlongAxis = [voxel_size_m](float min_center, float max_center) {
      const double span = static_cast<double>(max_center) - static_cast<double>(min_center);
      return static_cast<int>(std::floor(span / static_cast<double>(voxel_size_m) + 0.5)) + 1;
    };

  const int width = cellsAlongAxis(bounds.min_x, bounds.max_x);
  const int height = cellsAlongAxis(bounds.min_y, bounds.max_y);
  const float origin_x = bounds.min_x - 0.5f * voxel_size_m;
  const float origin_y = bounds.min_y - 0.5f * voxel_size_m;

  constexpr uint8_t kOccupied = 0;
  constexpr uint8_t kFree = 254;
  constexpr uint8_t kUnknown = 205;
  std::vector<uint8_t> image(static_cast<size_t>(width * height), kUnknown);

  for (int y = 0; y < height; ++y) {
    const float sample_y = bounds.min_y + static_cast<float>(y) * voxel_size_m;
    for (int x = 0; x < width; ++x) {
      const float sample_x = bounds.min_x + static_cast<float>(x) * voxel_size_m;
      const nvblox::Vector3f sample_position(
        sample_x, sample_y, static_cast<float>(slice_height_m));

      const nvblox::EsdfVoxel *voxel = nullptr;
      if (!getVoxelForSampling(sample_position, &voxel) || voxel == nullptr ||
        !voxel->observed)
      {
        continue;
      }

      const float unsigned_distance_m =
        std::sqrt(std::max(voxel->squared_distance_vox, 0.0f)) * voxel_size_m;
      const float signed_distance_m = voxel->is_inside ? -unsigned_distance_m : unsigned_distance_m;
      image[static_cast<size_t>(y * width + x)] =
        (signed_distance_m <= 0.0f) ? kOccupied : kFree;
    }
  }

  const std::string stem = "esdf_slice_z_" + sanitizeForFilename(slice_height_m) + "_occupancy";
  const auto image_path = output_dir / (stem + ".pgm");
  const auto yaml_path = output_dir / (stem + ".yaml");

  std::ofstream image_file(image_path, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!image_file.is_open()) {
    throw std::runtime_error("failed to open output file: " + image_path.string());
  }
  image_file << "P5\n" << width << " " << height << "\n255\n";
  for (int row = height - 1; row >= 0; --row) {
    const char *row_ptr = reinterpret_cast<const char *>(&image[static_cast<size_t>(row * width)]);
    image_file.write(row_ptr, width);
  }

  std::ofstream yaml_file(yaml_path, std::ios::out | std::ios::trunc);
  if (!yaml_file.is_open()) {
    throw std::runtime_error("failed to open output file: " + yaml_path.string());
  }
  yaml_file << "image: \"" << image_path.filename().string() << "\"\n";
  yaml_file << "resolution: " << voxel_size_m << "\n";
  yaml_file << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
  yaml_file << "negate: 0\n";
  yaml_file << "occupied_thresh: 0.65\n";
  yaml_file << "free_thresh: 0.196\n";
  yaml_file << "mode: trinary\n";

  result->esdf_slice_occupancy_image_path = image_path;
  result->esdf_slice_occupancy_yaml_path = yaml_path;
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
  if (config.crop_border_px < 0) {
    throw std::runtime_error("crop_border_px must be non-negative");
  }
  if (config.truncation_distance_vox <= 0.0) {
    throw std::runtime_error("truncation_distance_vox must be positive");
  }
  if (config.max_weight <= 0.0) {
    throw std::runtime_error("max_weight must be positive");
  }
  if (config.mesh_min_weight < 0.0) {
    throw std::runtime_error("mesh_min_weight must be non-negative");
  }
  if (config.esdf_max_distance_m <= 0.0) {
    throw std::runtime_error("esdf_max_distance_m must be positive");
  }
  if (config.esdf_min_weight < 0.0) {
    throw std::runtime_error("esdf_min_weight must be non-negative");
  }
  if (config.esdf_max_site_distance_vox <= 0.0) {
    throw std::runtime_error("esdf_max_site_distance_vox must be positive");
  }
  if (!std::isfinite(config.max_world_z_m) && !std::isinf(config.max_world_z_m)) {
    throw std::runtime_error("max_world_z_m must be finite or +inf");
  }

  FusionResult result;

  nvblox::Mapper mapper(
    static_cast<float>(config.voxel_size_m),
    nvblox::BlockMemoryPoolParams(),
    nvblox::ProjectiveLayerType::kTsdf);

  nvblox::MapperParams mapper_params;
  mapper_params.projective_integrator_params.projective_integrator_max_integration_distance_m =
    static_cast<float>(config.max_depth_m);
  mapper_params.projective_integrator_params.projective_integrator_truncation_distance_vox =
    static_cast<float>(config.truncation_distance_vox);
  mapper_params.projective_integrator_params.projective_integrator_max_weight =
    static_cast<float>(config.max_weight);
  mapper_params.mesh_integrator_params.mesh_integrator_min_weight =
    static_cast<float>(config.mesh_min_weight);
  mapper_params.esdf_integrator_params.esdf_integrator_max_distance_m =
    static_cast<float>(config.esdf_max_distance_m);
  mapper_params.esdf_integrator_params.esdf_integrator_min_weight =
    static_cast<float>(config.esdf_min_weight);
  mapper_params.esdf_integrator_params.esdf_integrator_max_site_distance_vox =
    static_cast<float>(config.esdf_max_site_distance_vox);
  if (config.esdf_slice_height_m.has_value()) {
    const float slice_height_m = static_cast<float>(*config.esdf_slice_height_m);
    const float half_voxel_height_m = static_cast<float>(config.voxel_size_m * 0.5);
    mapper_params.esdf_integrator_params.esdf_slice_min_height =
      slice_height_m - half_voxel_height_m;
    mapper_params.esdf_integrator_params.esdf_slice_max_height =
      slice_height_m + half_voxel_height_m;
    mapper_params.esdf_integrator_params.esdf_slice_height = slice_height_m;
  }
  mapper.setMapperParams(mapper_params);
  mapper.tsdf_integrator().weighting_function_type(
    nvblox::WeightingFunctionType::kInverseSquareTsdfDistancePenalty);
  mapper.tsdf_integrator().max_weight(static_cast<float>(config.max_weight));
  mapper.color_integrator().sphere_tracing_ray_subsampling_factor(1);
  mapper.color_mesh_integrator().weld_vertices(true);

  const int stride = std::max(1, config.pixel_stride);
  const int crop = config.crop_border_px;
  const int full_width = session.camera.width;
  const int full_height = session.camera.height;
  if ((crop * 2) >= full_width || (crop * 2) >= full_height) {
    throw std::runtime_error(
      "crop_border_px is too large for camera intrinsics: " + std::to_string(crop));
  }
  const int roi_width = full_width - 2 * crop;
  const int roi_height = full_height - 2 * crop;
  const int camera_width = (roi_width + stride - 1) / stride;
  const int camera_height = (roi_height + stride - 1) / stride;
  const auto distortion_params = toNvbloxDistortion(session.camera);
  nvblox::Camera camera(
    static_cast<float>(session.camera.fx / stride),
    static_cast<float>(session.camera.fy / stride),
    static_cast<float>((session.camera.cx - crop) / stride),
    static_cast<float>((session.camera.cy - crop) / stride),
    camera_width,
    camera_height,
    distortion_params);

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
    if ((crop * 2) >= rows || (crop * 2) >= cols) {
      throw std::runtime_error(
        "crop_border_px is too large for image size: " + std::to_string(crop));
    }
    const int roi_rows = rows - 2 * crop;
    const int roi_cols = cols - 2 * crop;
    const int sampled_rows = (roi_rows + stride - 1) / stride;
    const int sampled_cols = (roi_cols + stride - 1) / stride;

    std::vector<float> depth_buffer(static_cast<size_t>(sampled_rows * sampled_cols), 0.0f);
    std::vector<nvblox::Color> color_buffer(static_cast<size_t>(sampled_rows * sampled_cols));

    for (int out_y = 0; out_y < sampled_rows; ++out_y) {
      const int src_y = std::min(crop + out_y * stride, rows - crop - 1);
      for (int out_x = 0; out_x < sampled_cols; ++out_x) {
        const int src_x = std::min(crop + out_x * stride, cols - crop - 1);
        const size_t linear_idx = static_cast<size_t>(out_y * sampled_cols + out_x);

        const double depth_m = depthMetersAt(depth, src_y, src_x, config.depth_scale);
        if (std::isfinite(depth_m) && depth_m >= config.min_depth_m && depth_m <= config.max_depth_m) {
          bool keep_point = true;
          if (std::isfinite(config.max_world_z_m)) {
            const double point_x =
              (static_cast<double>(src_x) - session.camera.cx) * depth_m / session.camera.fx;
            const double point_y =
              (static_cast<double>(src_y) - session.camera.cy) * depth_m / session.camera.fy;
            const cv::Vec3d point_in_camera(point_x, point_y, depth_m);
            const cv::Vec3d point_in_world = transformPoint(world_T_camera, point_in_camera);
            keep_point = point_in_world[2] <= config.max_world_z_m;
          }
          if (keep_point) {
            depth_buffer[linear_idx] = static_cast<float>(depth_m);
            ++result.raw_points_considered;
          }
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

  const bool need_full_esdf = config.export_esdf;
  const bool need_slice_esdf = config.esdf_slice_height_m.has_value();
  std::filesystem::create_directories(output_dir);

  if (need_full_esdf) {
    mapper.updateEsdf(nvblox::UpdateFullLayer::kYes);
    result.esdf_path = output_dir / "fused_esdf.ply";
    if (!nvblox::io::outputVoxelLayerToPly(
          mapper.esdf_layer(),
          result.esdf_path.string())) {
      throw std::runtime_error("nvblox failed to write ESDF PLY: " + result.esdf_path.string());
    }
    result.esdf_block_count = mapper.esdf_layer().size();
  }
  if (need_slice_esdf && !need_full_esdf) {
    mapper.updateEsdfSlice(nvblox::UpdateFullLayer::kYes);
  }
  if (need_slice_esdf) {
    exportEsdfSliceOccupancyMap(
      mapper.esdf_layer(), *config.esdf_slice_height_m, output_dir, &result);
  }

  mapper.updateColorMesh(nvblox::UpdateFullLayer::kYes);

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
