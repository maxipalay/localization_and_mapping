#include "online_mapping_logger/session_writer.hpp"

#include "online_mapping_logger/utils.hpp"
#include "session_writer_detail.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace online_mapping_logger
{

void SessionWriter::prepareSessionLayout_()
{
  const std::filesystem::path root(config_.output_root);
  const std::filesystem::path session_path = root / config_.session_name;
  std::error_code ec;

  if (std::filesystem::exists(session_path, ec)) {
    if (!config_.overwrite_existing_session) {
      throw std::runtime_error("session directory already exists: " + session_path.string());
    }
    std::filesystem::remove_all(session_path, ec);
    if (ec) {
      throw std::runtime_error("failed to remove existing session directory: " + session_path.string());
    }
  }

  rgb_dir_ = (session_path / "rgb").string();
  depth_dir_ = (session_path / "depth").string();
  tags_dir_ = (session_path / "tags").string();
  keyframes_dir_ = (session_path / "keyframes").string();
  calibration_dir_ = (session_path / "calibration").string();
  session_dir_ = session_path.string();

  std::filesystem::create_directories(session_path);
  std::filesystem::create_directories(rgb_dir_);
  std::filesystem::create_directories(depth_dir_);
  std::filesystem::create_directories(tags_dir_);
  std::filesystem::create_directories(keyframes_dir_);
  std::filesystem::create_directories(calibration_dir_);
}

void SessionWriter::openManifest_()
{
  manifest_path_ = std::filesystem::path(session_dir_) / "keyframe_manifest.csv";
  rewriteManifest_();
}

void SessionWriter::writeSessionMetadata_()
{
  const auto metadata_path = std::filesystem::path(session_dir_) / "session_metadata.yaml";
  std::ofstream os(metadata_path, std::ios::out | std::ios::trunc);
  if (!os.is_open()) {
    throw std::runtime_error("failed to open metadata file: " + metadata_path.string());
  }

  os << "session_name: " << detail::yamlDoubleQuoted(config_.session_name) << "\n";
  os << "created_at_utc: " << detail::yamlDoubleQuoted(nowUtcString()) << "\n";
  os << "dataset_version: 3\n";
  os << "frames:\n";
  os << "  body: " << detail::yamlDoubleQuoted(config_.body_frame_id) << "\n";
  os << "topics:\n";
  os << "  rgb_image: " << detail::yamlDoubleQuoted(config_.rgb_image_topic) << "\n";
  os << "  rgb_camera_info: " << detail::yamlDoubleQuoted(config_.rgb_camera_info_topic) << "\n";
  os << "  depth_image: " << detail::yamlDoubleQuoted(config_.depth_image_topic) << "\n";
  os << "  depth_camera_info: " << detail::yamlDoubleQuoted(config_.depth_camera_info_topic) << "\n";
  os << "  keyframe: " << detail::yamlDoubleQuoted(config_.keyframe_topic) << "\n";
  os << "  optimization_result: " << detail::yamlDoubleQuoted(config_.optimization_result_topic) << "\n";
  os << "  tag: " << detail::yamlDoubleQuoted(config_.tag_topic) << "\n";
  os << "matching:\n";
  os << "  rgb_tolerance_ns: " << config_.rgb_match_tolerance_ns << "\n";
  os << "  depth_tolerance_ns: " << config_.depth_match_tolerance_ns << "\n";
  os << "  tag_tolerance_ns: " << config_.tag_match_tolerance_ns << "\n";
  os << "  buffer_duration_ns: " << config_.buffer_duration_ns << "\n";
  os << "  pending_timeout_ns: " << config_.pending_timeout_ns << "\n";
  os << "requirements:\n";
  os << "  require_depth: " << (config_.require_depth ? "true" : "false") << "\n";
  os << "  require_tags: " << (config_.require_tags ? "true" : "false") << "\n";
#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
  os << "tag_message_type: \"apriltag_msgs/msg/AprilTagDetectionArray\"\n";
#else
  os << "tag_message_type: \"unavailable_at_build_time\"\n";
#endif
  os << "files:\n";
  os << "  manifest: \"keyframe_manifest.csv\"\n";
  os << "  rgb_dir: \"rgb\"\n";
  os << "  depth_dir: \"depth\"\n";
  os << "  tags_dir: \"tags\"\n";
  os << "  keyframes_dir: \"keyframes\"\n";
  os << "  calibration_dir: \"calibration\"\n";
  os << "  rgb_body_to_camera: \"calibration/body_to_rgb_camera.yaml\"\n";
  os << "  depth_body_to_camera: \"calibration/body_to_depth_camera.yaml\"\n";
}

void SessionWriter::copyTagPriorFile_()
{
  const auto dst = std::filesystem::path(session_dir_) / "tag_priors.yaml";
  if (config_.tag_prior_source_path.empty()) {
    std::ofstream os(dst, std::ios::out | std::ios::trunc);
    os << "# Tag priors were not provided at logging time.\n";
    return;
  }

  std::error_code ec;
  std::filesystem::copy_file(
    config_.tag_prior_source_path,
    dst,
    std::filesystem::copy_options::overwrite_existing,
    ec);
  if (ec) {
    std::ofstream os(dst, std::ios::out | std::ios::trunc);
    os << "# Failed to copy source tag prior file: " << config_.tag_prior_source_path << "\n";
  }
}

}  // namespace online_mapping_logger
