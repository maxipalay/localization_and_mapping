#include "offline_dense_map_fusion/exporter.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

namespace offline_dense_map_fusion
{

namespace
{

cv::Vec4d quaternionFromRotation(const cv::Matx33d &R)
{
  const double trace = R(0, 0) + R(1, 1) + R(2, 2);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (R(2, 1) - R(1, 2)) / s;
    y = (R(0, 2) - R(2, 0)) / s;
    z = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
    const double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
    w = (R(2, 1) - R(1, 2)) / s;
    x = 0.25 * s;
    y = (R(0, 1) + R(1, 0)) / s;
    z = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    const double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
    w = (R(0, 2) - R(2, 0)) / s;
    x = (R(0, 1) + R(1, 0)) / s;
    y = 0.25 * s;
    z = (R(1, 2) + R(2, 1)) / s;
  } else {
    const double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
    w = (R(1, 0) - R(0, 1)) / s;
    x = (R(0, 2) + R(2, 0)) / s;
    y = (R(1, 2) + R(2, 1)) / s;
    z = 0.25 * s;
  }

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  return cv::Vec4d(x / norm, y / norm, z / norm, w / norm);
}

void writePoseCsvRow(std::ostream &os, const Pose &pose)
{
  const cv::Vec4d q = quaternionFromRotation(pose.rotation);
  os
    << pose.translation[0] << ","
    << pose.translation[1] << ","
    << pose.translation[2] << ","
    << q[0] << ","
    << q[1] << ","
    << q[2] << ","
    << q[3];
}

}  // namespace

void writeFusionOutputs(
  const SessionData &session,
  const Extrinsics &extrinsics,
  const FusionConfig &config,
  const FusionResult &result,
  const std::filesystem::path &output_dir)
{
  std::filesystem::create_directories(output_dir);

  const auto poses_path = output_dir / "camera_poses.csv";
  std::ofstream poses(poses_path, std::ios::out | std::ios::trunc);
  if (!poses.is_open()) {
    throw std::runtime_error("failed to open output file: " + poses_path.string());
  }

  poses
    << "kf_id,stamp_ns,cam_px,cam_py,cam_pz,cam_qx,cam_qy,cam_qz,cam_qw\n";
  for (size_t i = 0; i < result.world_T_camera_by_kf.size(); ++i) {
    poses << result.world_T_camera_by_kf[i].first << "," << session.frames[i].stamp_ns << ",";
    writePoseCsvRow(poses, result.world_T_camera_by_kf[i].second);
    poses << "\n";
  }

  const auto summary_path = output_dir / "fusion_summary.yaml";
  std::ofstream summary(summary_path, std::ios::out | std::ios::trunc);
  if (!summary.is_open()) {
    throw std::runtime_error("failed to open output file: " + summary_path.string());
  }

  summary << "session_dir: \"" << session.session_dir.string() << "\"\n";
  summary << "session_name: \"" << session.session_name << "\"\n";
  summary << "body_frame_id: \"" << session.body_frame_id << "\"\n";
  summary << "camera_frame_id: \"" << extrinsics.camera_frame_id << "\"\n";
  summary << "fusion_backend: \"nvblox_tsdf\"\n";
  summary << "frame_count: " << result.frames_fused << "\n";
  summary << "raw_points_considered: " << result.raw_points_considered << "\n";
  summary << "mesh_path: \"" << result.mesh_path.filename().string() << "\"\n";
  summary << "mesh_block_count: " << result.mesh_block_count << "\n";
  summary << "voxel_size_m: " << config.voxel_size_m << "\n";
  summary << "depth_scale: " << config.depth_scale << "\n";
  summary << "min_depth_m: " << config.min_depth_m << "\n";
  summary << "max_depth_m: " << config.max_depth_m << "\n";
  summary << "pixel_stride: " << config.pixel_stride << "\n";
  summary << "intrinsics:\n";
  summary << "  width: " << session.camera.width << "\n";
  summary << "  height: " << session.camera.height << "\n";
  summary << "  fx: " << session.camera.fx << "\n";
  summary << "  fy: " << session.camera.fy << "\n";
  summary << "  cx: " << session.camera.cx << "\n";
  summary << "  cy: " << session.camera.cy << "\n";
}

}  // namespace offline_dense_map_fusion
