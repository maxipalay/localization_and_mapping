#include "online_mapping_logger/session_writer.hpp"

#include "session_writer_detail.hpp"

#include <fstream>
#include <stdexcept>

namespace online_mapping_logger
{

void SessionWriter::appendManifestLine_(
  const PendingKeyframe &pending,
  const std::filesystem::path &rgb_path,
  const std::filesystem::path &depth_path,
  const std::filesystem::path &tags_path,
  const std::filesystem::path &keyframe_meta_path) const
{
  const auto rgb_rel = detail::relativeTo(rgb_path, session_dir_).generic_string();
  const auto depth_rel =
    depth_path.empty() ? std::string() : detail::relativeTo(depth_path, session_dir_).generic_string();
  const auto tags_rel =
    tags_path.empty() ? std::string() : detail::relativeTo(tags_path, session_dir_).generic_string();
  const auto meta_rel = detail::relativeTo(keyframe_meta_path, session_dir_).generic_string();

  const size_t tag_count = pending.tag_poses.size();

  std::ofstream manifest(manifest_path_, std::ios::out | std::ios::app);
  if (!manifest.is_open()) {
    throw std::runtime_error("failed to open manifest for append: " + manifest_path_.string());
  }

  manifest
    << pending.keyframe.kf_id << ","
    << pending.keyframe_stamp_ns << ","
    << pending.rgb_stamp_ns << ","
    << pending.depth_stamp_ns << ","
    << pending.tags_stamp_ns << ","
    << detail::csvEscape(rgb_rel) << ","
    << detail::csvEscape(depth_rel) << ","
    << detail::csvEscape(tags_rel) << ","
    << detail::csvEscape(meta_rel) << ","
    << pending.keyframe.pose_odom_body.position.x << ","
    << pending.keyframe.pose_odom_body.position.y << ","
    << pending.keyframe.pose_odom_body.position.z << ","
    << pending.keyframe.pose_odom_body.orientation.x << ","
    << pending.keyframe.pose_odom_body.orientation.y << ","
    << pending.keyframe.pose_odom_body.orientation.z << ","
    << pending.keyframe.pose_odom_body.orientation.w << ","
    << pending.opt_result.pose_wb_opt.position.x << ","
    << pending.opt_result.pose_wb_opt.position.y << ","
    << pending.opt_result.pose_wb_opt.position.z << ","
    << pending.opt_result.pose_wb_opt.orientation.x << ","
    << pending.opt_result.pose_wb_opt.orientation.y << ","
    << pending.opt_result.pose_wb_opt.orientation.z << ","
    << pending.opt_result.pose_wb_opt.orientation.w << ","
    << pending.keyframe.track_ids.size() << ","
    << tag_count << "\n";
}

void SessionWriter::rewriteManifest_() const
{
  std::ofstream manifest(manifest_path_, std::ios::out | std::ios::trunc);
  if (!manifest.is_open()) {
    throw std::runtime_error("failed to open manifest: " + manifest_path_.string());
  }

  manifest
    << "kf_id,keyframe_stamp_ns,rgb_stamp_ns,depth_stamp_ns,tags_stamp_ns,"
    << "rgb_path,depth_path,tags_path,keyframe_meta_path,"
    << "frontend_px,frontend_py,frontend_pz,frontend_qx,frontend_qy,frontend_qz,frontend_qw,"
    << "opt_body_px,opt_body_py,opt_body_pz,opt_body_qx,opt_body_qy,opt_body_qz,opt_body_qw,"
    << "track_count,tag_count\n";
  manifest.close();

  for (const auto &[kf_id, record] : records_) {
    appendManifestLine_(
      record.pending,
      record.rgb_path,
      record.depth_path,
      record.tags_path,
      record.keyframe_meta_path);
  }
}

}  // namespace online_mapping_logger
