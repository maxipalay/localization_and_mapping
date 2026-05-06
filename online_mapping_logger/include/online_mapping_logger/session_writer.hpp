#pragma once

#include "online_mapping_logger/types.hpp"

#include <filesystem>
#include <map>
#include <string>

namespace online_mapping_logger
{

class SessionWriter
{
public:
  explicit SessionWriter(const LoggerConfig &config);

  const std::string &sessionDir() const;
  void writeCameraInfo(const CameraInfoMsg &msg, const std::string &filename);
  void writeBodyToCameraExtrinsics(
    const std::string &body_frame_id,
    const std::string &camera_frame_id,
    const geometry_msgs::msg::TransformStamped &transform,
    const std::string &filename);
  void ingestOptimizationResult(const OptimizationResultMsg &msg);
  void writeRecord(const CompletedRecord &record);

private:
  struct LatestPoseEntry
  {
    int64_t stamp_ns{0};
    geometry_msgs::msg::Pose pose_wb;
    geometry_msgs::msg::Pose pose_wc;
  };

  struct StoredRecord
  {
    PendingKeyframe pending;
    std::filesystem::path rgb_path;
    std::filesystem::path depth_path;
    std::filesystem::path tags_path;
    std::filesystem::path keyframe_meta_path;
    int64_t latest_pose_stamp_ns{0};
  };

  void prepareSessionLayout_();
  void openManifest_();
  void writeSessionMetadata_();
  void copyTagPriorFile_();
  void rewriteManifest_() const;
  void applyLatestPoseIfAvailable_(PendingKeyframe &pending, int64_t *applied_stamp_ns) const;
  void updateStoredRecordPose_(StoredRecord &record, const LatestPoseEntry &entry);
  void persistStoredRecord_(const StoredRecord &record) const;

  std::filesystem::path writeImage_(
    const ImageMsg::ConstSharedPtr &msg,
    const std::filesystem::path &path) const;
  std::filesystem::path writeKeyframeMetadata_(
    const PendingKeyframe &pending,
    const std::filesystem::path &rgb_path,
    const std::filesystem::path &depth_path,
    const std::filesystem::path &tags_path,
    const std::filesystem::path &path) const;
  std::filesystem::path writeTags_(
    const PendingKeyframe &pending,
    const std::filesystem::path &path) const;
  void appendManifestLine_(
    const PendingKeyframe &pending,
    const std::filesystem::path &rgb_path,
    const std::filesystem::path &depth_path,
    const std::filesystem::path &tags_path,
    const std::filesystem::path &keyframe_meta_path) const;
  std::string imageExtensionFor_(const ImageMsg &msg, bool is_depth) const;

  LoggerConfig config_;
  std::string session_dir_;
  std::string rgb_dir_;
  std::string depth_dir_;
  std::string tags_dir_;
  std::string keyframes_dir_;
  std::string calibration_dir_;
  std::filesystem::path manifest_path_;
  std::map<uint64_t, LatestPoseEntry> latest_pose_by_kf_id_;
  std::map<uint64_t, StoredRecord> records_;
};

}  // namespace online_mapping_logger
