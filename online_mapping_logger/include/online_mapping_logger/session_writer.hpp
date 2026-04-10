#pragma once

#include "online_mapping_logger/types.hpp"

#include <filesystem>
#include <fstream>
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
  void writeRecord(const CompletedRecord &record);

private:
  void prepareSessionLayout_();
  void openManifest_();
  void writeSessionMetadata_();
  void copyTagPriorFile_();

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
    const std::filesystem::path &keyframe_meta_path);
  std::string imageExtensionFor_(const ImageMsg &msg, bool is_depth) const;

  LoggerConfig config_;
  std::string session_dir_;
  std::string rgb_dir_;
  std::string depth_dir_;
  std::string tags_dir_;
  std::string keyframes_dir_;
  std::string calibration_dir_;
  std::ofstream manifest_;
};

}  // namespace online_mapping_logger
