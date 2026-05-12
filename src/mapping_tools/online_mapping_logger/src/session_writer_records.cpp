#include "online_mapping_logger/session_writer.hpp"

#include "online_mapping_logger/utils.hpp"
#include "session_writer_detail.hpp"

#include <fstream>
#include <stdexcept>

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#endif

namespace online_mapping_logger
{

void SessionWriter::ingestOptimizationResult(const OptimizationResultMsg &msg)
{
  const int64_t stamp_ns = stampToNs(msg.header.stamp);

  auto ingest_pose = [this, stamp_ns](
                       uint64_t kf_id,
                       const geometry_msgs::msg::Pose &pose_wb,
                       const geometry_msgs::msg::Pose &pose_wc) {
      auto &entry = latest_pose_by_kf_id_[kf_id];
      if (stamp_ns < entry.stamp_ns) {
        return;
      }
      entry.stamp_ns = stamp_ns;
      entry.pose_wb = pose_wb;
      entry.pose_wc = pose_wc;
    };

  ingest_pose(msg.kf_id, msg.pose_wb_opt, msg.pose_wc_opt);
  for (const auto &active_pose : msg.active_keyframe_poses) {
    ingest_pose(active_pose.kf_id, active_pose.pose_wb_opt, active_pose.pose_wc_opt);
  }

  bool manifest_changed = false;
  for (auto &[kf_id, record] : records_) {
    const auto pose_it = latest_pose_by_kf_id_.find(kf_id);
    if (pose_it == latest_pose_by_kf_id_.end() ||
        pose_it->second.stamp_ns <= record.latest_pose_stamp_ns) {
      continue;
    }

    updateStoredRecordPose_(record, pose_it->second);
    persistStoredRecord_(record);
    manifest_changed = true;
  }

  if (manifest_changed) {
    rewriteManifest_();
  }
}

void SessionWriter::writeRecord(const CompletedRecord &record)
{
  PendingKeyframe pending = record.pending;
  int64_t applied_pose_stamp_ns = 0;
  applyLatestPoseIfAvailable_(pending, &applied_pose_stamp_ns);

  const auto prefix = formatKfId(pending.keyframe.kf_id);
  const auto rgb_path = writeImage_(
    pending.rgb_msg,
    std::filesystem::path(rgb_dir_) / (prefix + imageExtensionFor_(*pending.rgb_msg, false)));

  std::filesystem::path depth_path;
  if (pending.have_depth && pending.depth_msg) {
    depth_path = writeImage_(
      pending.depth_msg,
      std::filesystem::path(depth_dir_) /
        (prefix + imageExtensionFor_(*pending.depth_msg, true)));
  }

  std::filesystem::path tags_path;
  if (pending.have_tags) {
    tags_path = writeTags_(pending, std::filesystem::path(tags_dir_) / (prefix + ".yaml"));
  }

  const auto keyframe_meta_path = writeKeyframeMetadata_(
    pending,
    rgb_path,
    depth_path,
    tags_path,
    std::filesystem::path(keyframes_dir_) / (prefix + ".yaml"));

  StoredRecord stored;
  stored.pending = std::move(pending);
  stored.pending.rgb_msg.reset();
  stored.pending.depth_msg.reset();
  stored.pending.tags_msg.reset();
  stored.rgb_path = rgb_path;
  stored.depth_path = depth_path;
  stored.tags_path = tags_path;
  stored.keyframe_meta_path = keyframe_meta_path;
  stored.latest_pose_stamp_ns = applied_pose_stamp_ns;

  records_[stored.pending.keyframe.kf_id] = std::move(stored);
  rewriteManifest_();
}

std::filesystem::path SessionWriter::writeKeyframeMetadata_(
  const PendingKeyframe &pending,
  const std::filesystem::path &rgb_path,
  const std::filesystem::path &depth_path,
  const std::filesystem::path &tags_path,
  const std::filesystem::path &path) const
{
  std::ofstream os(path, std::ios::out | std::ios::trunc);
  if (!os.is_open()) {
    throw std::runtime_error("failed to open keyframe metadata file: " + path.string());
  }

  os << "kf_id: " << pending.keyframe.kf_id << "\n";
  os << "header_stamp_ns: " << pending.keyframe_stamp_ns << "\n";
  os << "header_frame_id: " << detail::yamlDoubleQuoted(pending.keyframe.header.frame_id) << "\n";
  os << "t_start: " << pending.keyframe.t_start << "\n";
  os << "t_end: " << pending.keyframe.t_end << "\n";
  os << "kf_reason_mask: " << pending.keyframe.kf_reason_mask << "\n";
  os << "has_vo_between: " << ((pending.keyframe.has_vo_between != 0U) ? "true" : "false") << "\n";
  os << "has_imu: " << static_cast<int>(pending.keyframe.has_imu) << "\n";
  os << "pim_bytes_hex: " << detail::yamlDoubleQuoted(toHex(pending.keyframe.pim_bytes)) << "\n";
  detail::writePoseYaml(os, "frontend_pose_ob", pending.keyframe.pose_odom_body);
  if (pending.keyframe.has_vo_between != 0U) {
    detail::writePoseYaml(os, "between_pose_prev_curr_body", pending.keyframe.between_pose_prev_curr);
  }
  detail::writeIntervalHealthYaml(os, pending.keyframe.interval_health);
  detail::writePoseYaml(os, "optimized_pose_wb", pending.opt_result.pose_wb_opt);
  detail::writePoseYaml(os, "optimized_pose_wc", pending.opt_result.pose_wc_opt);
  os << "optimization:\n";
  os << "  header_stamp_ns: " << stampToNs(pending.opt_result.header.stamp) << "\n";
  os << "  header_frame_id: " << detail::yamlDoubleQuoted(pending.opt_result.header.frame_id) << "\n";
  os << "  t_s: " << pending.opt_result.t_s << "\n";
  os << "  has_velocity: " << (pending.opt_result.has_velocity ? "true" : "false") << "\n";
  detail::writeVector3Yaml(os, "  velocity_opt", pending.opt_result.velocity_opt);
  detail::writeVector3Yaml(os, "  accel_bias", pending.opt_result.accel_bias);
  detail::writeVector3Yaml(os, "  gyro_bias", pending.opt_result.gyro_bias);
  os << "  stats:\n";
  os << "    num_keyframes_in_window: " << pending.opt_result.stats.num_keyframes_in_window << "\n";
  os << "    num_landmarks_alive: " << pending.opt_result.stats.num_landmarks_alive << "\n";
  os << "    num_landmarks_created: " << pending.opt_result.stats.num_landmarks_created << "\n";
  os << "    num_stereo_factors_added: " << pending.opt_result.stats.num_stereo_factors_added << "\n";
  os << "    num_imu_factors_added: " << pending.opt_result.stats.num_imu_factors_added << "\n";
  os << "    num_between_factors_added: " << pending.opt_result.stats.num_between_factors_added << "\n";
  os << "    num_prior_factors_added: " << pending.opt_result.stats.num_prior_factors_added << "\n";
  os << "    had_vo_between_measurement: "
     << (pending.opt_result.stats.had_vo_between_measurement ? "true" : "false") << "\n";
  os << "    used_vo_between_factor: "
     << (pending.opt_result.stats.used_vo_between_factor ? "true" : "false") << "\n";
  os << "    skipped_vo_between_factor: "
     << (pending.opt_result.stats.skipped_vo_between_factor ? "true" : "false") << "\n";
  os << "    vo_between_quality: " << pending.opt_result.stats.vo_between_quality << "\n";
  os << "    vo_between_sigma_scale: " << pending.opt_result.stats.vo_between_sigma_scale << "\n";
  os << "    imu_only_update: " << (pending.opt_result.stats.imu_only_update ? "true" : "false") << "\n";
  os << "    update_iterations: " << pending.opt_result.stats.update_iterations << "\n";
  os << "    update_intermediate_steps: " << pending.opt_result.stats.update_intermediate_steps << "\n";
  os << "    update_nonlinear_variables: " << pending.opt_result.stats.update_nonlinear_variables << "\n";
  os << "    update_linear_variables: " << pending.opt_result.stats.update_linear_variables << "\n";
  os << "    final_error: " << pending.opt_result.stats.final_error << "\n";
  os << "    has_error_before: " << (pending.opt_result.stats.has_error_before ? "true" : "false") << "\n";
  os << "    error_before: " << pending.opt_result.stats.error_before << "\n";
  os << "    has_error_after: " << (pending.opt_result.stats.has_error_after ? "true" : "false") << "\n";
  os << "    error_after: " << pending.opt_result.stats.error_after << "\n";
  os << "    variables_relinearized: " << pending.opt_result.stats.variables_relinearized << "\n";
  os << "    variables_reeliminated: " << pending.opt_result.stats.variables_reeliminated << "\n";
  os << "    factors_recalculated: " << pending.opt_result.stats.factors_recalculated << "\n";
  os << "    cliques: " << pending.opt_result.stats.cliques << "\n";
  os << "  has_pose_wb_covariance: " << (pending.opt_result.has_pose_wb_covariance ? "true" : "false") << "\n";
  detail::writeScalarArray(os, "  pose_wb_covariance", pending.opt_result.pose_wb_covariance);
  os << "  has_velocity_covariance: " << (pending.opt_result.has_velocity_covariance ? "true" : "false") << "\n";
  detail::writeScalarArray(os, "  velocity_covariance", pending.opt_result.velocity_covariance);
  os << "  has_bias_covariance: " << (pending.opt_result.has_bias_covariance ? "true" : "false") << "\n";
  detail::writeScalarArray(os, "  bias_covariance", pending.opt_result.bias_covariance);

  os << "associated_files:\n";
  os << "  rgb_path: " << detail::yamlDoubleQuoted(detail::relativeTo(rgb_path, session_dir_).generic_string()) << "\n";
  os << "  depth_path: "
     << detail::yamlDoubleQuoted(
          depth_path.empty() ? std::string() : detail::relativeTo(depth_path, session_dir_).generic_string())
     << "\n";
  os << "  tags_path: "
     << detail::yamlDoubleQuoted(
          tags_path.empty() ? std::string() : detail::relativeTo(tags_path, session_dir_).generic_string())
     << "\n";
  os << "associated_stamps_ns:\n";
  os << "  rgb: " << pending.rgb_stamp_ns << "\n";
  os << "  depth: " << pending.depth_stamp_ns << "\n";
  os << "  tags: " << pending.tags_stamp_ns << "\n";
  os << "tag_pose_count: " << pending.tag_poses.size() << "\n";

  os << "tracks:\n";
  detail::writeScalarArray(os, "  track_ids", pending.keyframe.track_ids);
  detail::writeScalarArray(os, "  u_l", pending.keyframe.u_l);
  detail::writeScalarArray(os, "  v_l", pending.keyframe.v_l);
  detail::writeScalarArray(os, "  u_r", pending.keyframe.u_r);
  detail::writeScalarArray(os, "  v_r", pending.keyframe.v_r);
  detail::writeScalarArray(os, "  has_right", pending.keyframe.has_right);

  return path;
}

std::filesystem::path SessionWriter::writeTags_(
  const PendingKeyframe &pending,
  const std::filesystem::path &path) const
{
  std::ofstream os(path, std::ios::out | std::ios::trunc);
  if (!os.is_open()) {
    throw std::runtime_error("failed to open tag file: " + path.string());
  }

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
  os << "tag_message_type: \"apriltag_msgs/msg/AprilTagDetectionArray\"\n";
  os << "header_stamp_ns: " << pending.tags_stamp_ns << "\n";
  os << "header_frame_id: "
     << detail::yamlDoubleQuoted(pending.tags_msg ? pending.tags_msg->header.frame_id : std::string())
     << "\n";
  os << "source_message_count: " << (pending.tags_msg ? 1 : 0) << "\n";
  os << "detections:\n";
  for (const auto &tag_pose : pending.tag_poses) {
    os << "  - family: " << detail::yamlDoubleQuoted(tag_pose.family) << "\n";
    os << "    id: " << tag_pose.id << "\n";
    os << "    sample_count: " << tag_pose.sample_count << "\n";
    os << "    resolved_sample_count: " << tag_pose.resolved_sample_count << "\n";
    os << "    hamming: " << tag_pose.hamming << "\n";
    os << "    goodness: " << tag_pose.goodness << "\n";
    os << "    decision_margin: " << tag_pose.decision_margin << "\n";
    os << "    centre: [" << tag_pose.centre[0] << ", " << tag_pose.centre[1] << "]\n";
    os << "    homography: [";
    for (size_t i = 0; i < tag_pose.homography.size(); ++i) {
      if (i > 0) {
        os << ", ";
      }
      os << tag_pose.homography[i];
    }
    os << "]\n";
    os << "    corners:\n";
    for (const auto &corner : tag_pose.corners) {
      os << "      - [" << corner[0] << ", " << corner[1] << "]\n";
    }
    os << "    tf_pose:\n";
    os << "      available: " << (tag_pose.pose_available ? "true" : "false") << "\n";
    os << "      detection_frame_id: " << detail::yamlDoubleQuoted(tag_pose.detection_frame_id) << "\n";
    os << "      parent_frame_id: " << detail::yamlDoubleQuoted(tag_pose.parent_frame_id) << "\n";
    os << "      child_frame_id: " << detail::yamlDoubleQuoted(tag_pose.child_frame_id) << "\n";
    os << "      lookup_stamp_ns: " << tag_pose.lookup_stamp_ns << "\n";
    if (tag_pose.pose_available) {
      os << "      translation: ["
         << tag_pose.transform.transform.translation.x << ", "
         << tag_pose.transform.transform.translation.y << ", "
         << tag_pose.transform.transform.translation.z << "]\n";
      os << "      orientation_xyzw: ["
         << tag_pose.transform.transform.rotation.x << ", "
         << tag_pose.transform.transform.rotation.y << ", "
         << tag_pose.transform.transform.rotation.z << ", "
         << tag_pose.transform.transform.rotation.w << "]\n";
    } else {
      os << "      lookup_error: " << detail::yamlDoubleQuoted(tag_pose.lookup_error) << "\n";
    }
  }
#else
  os << "tag_message_type: \"unavailable_at_build_time\"\n";
  os << "header_stamp_ns: " << pending.tags_stamp_ns << "\n";
  os << "detections: []\n";
#endif
  return path;
}

void SessionWriter::applyLatestPoseIfAvailable_(
  PendingKeyframe &pending,
  int64_t *applied_stamp_ns) const
{
  if (applied_stamp_ns) {
    *applied_stamp_ns = 0;
  }

  const auto it = latest_pose_by_kf_id_.find(pending.keyframe.kf_id);
  if (it == latest_pose_by_kf_id_.end()) {
    return;
  }

  pending.opt_result.pose_wb_opt = it->second.pose_wb;
  pending.opt_result.pose_wc_opt = it->second.pose_wc;
  if (applied_stamp_ns) {
    *applied_stamp_ns = it->second.stamp_ns;
  }
}

void SessionWriter::updateStoredRecordPose_(StoredRecord &record, const LatestPoseEntry &entry)
{
  record.pending.opt_result.pose_wb_opt = entry.pose_wb;
  record.pending.opt_result.pose_wc_opt = entry.pose_wc;
  record.latest_pose_stamp_ns = entry.stamp_ns;
}

void SessionWriter::persistStoredRecord_(const StoredRecord &record) const
{
  writeKeyframeMetadata_(
    record.pending,
    record.rgb_path,
    record.depth_path,
    record.tags_path,
    record.keyframe_meta_path);
}

}  // namespace online_mapping_logger
