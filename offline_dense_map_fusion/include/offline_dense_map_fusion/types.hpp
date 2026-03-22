#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace offline_dense_map_fusion
{

struct Pose
{
  cv::Matx33d rotation{cv::Matx33d::eye()};
  cv::Vec3d translation{0.0, 0.0, 0.0};
};

struct CameraIntrinsics
{
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string frame_id;
};

struct Extrinsics
{
  std::string body_frame_id{"body"};
  std::string camera_frame_id{"oak_left_optical"};
  Pose body_T_camera;
};

struct SessionFrame
{
  uint64_t kf_id{0};
  int64_t stamp_ns{0};
  std::filesystem::path rgb_path;
  std::filesystem::path depth_path;
  Pose world_T_body;
};

struct SessionData
{
  std::filesystem::path session_dir;
  std::string session_name;
  std::string body_frame_id{"body"};
  CameraIntrinsics camera;
  std::vector<SessionFrame> frames;
};

struct FusionConfig
{
  double voxel_size_m{0.03};
  double depth_scale{0.001};
  double min_depth_m{0.2};
  double max_depth_m{5.0};
  int pixel_stride{2};
  int crop_border_px{0};
  double truncation_distance_vox{4.0};
  double max_weight{5.0};
  double mesh_min_weight{1e-4};
};

struct FusionResult
{
  std::filesystem::path mesh_path;
  std::vector<std::pair<uint64_t, Pose>> world_T_camera_by_kf;
  size_t frames_fused{0};
  size_t raw_points_considered{0};
  size_t mesh_block_count{0};
};

}  // namespace offline_dense_map_fusion
