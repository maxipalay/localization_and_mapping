#include "online_mapping_logger/session_writer.hpp"

#include "online_mapping_logger/utils.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <filesystem>
#include <stdexcept>

namespace
{

template <typename TContainer>
void writeScalarArray(std::ostream &os, const std::string &key, const TContainer &values)
{
  os << key << ": [";
  size_t i = 0;
  for (const auto &value : values) {
    if (i > 0) {
      os << ", ";
    }
    os << +value;
    ++i;
  }
  os << "]\n";
}

void writePoseYaml(std::ostream &os, const std::string &key, const geometry_msgs::msg::Pose &pose)
{
  os << key << ":\n";
  os << "  position: [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "]\n";
  os << "  orientation_xyzw: ["
     << pose.orientation.x << ", "
     << pose.orientation.y << ", "
     << pose.orientation.z << ", "
     << pose.orientation.w << "]\n";
}

std::string csvEscape(const std::string &value)
{
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }

  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(c);
    }
  }
  escaped.push_back('"');
  return escaped;
}

std::string yamlDoubleQuoted(const std::string &value)
{
  std::string escaped = "\"";
  escaped.reserve(value.size() + 2);
  for (const char c : value) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped.push_back(c);
        break;
    }
  }
  escaped.push_back('"');
  return escaped;
}

std::filesystem::path relativeTo(
  const std::filesystem::path &path,
  const std::filesystem::path &base)
{
  std::error_code ec;
  auto rel = std::filesystem::relative(path, base, ec);
  return ec ? path.filename() : rel;
}

}  // namespace

namespace online_mapping_logger
{

SessionWriter::SessionWriter(const LoggerConfig &config)
: config_(config)
{
  prepareSessionLayout_();
  openManifest_();
  writeSessionMetadata_();
  copyTagPriorFile_();
}

const std::string &SessionWriter::sessionDir() const
{
  return session_dir_;
}

void SessionWriter::writeCameraInfo(const CameraInfoMsg &msg, const std::string &filename)
{
  const auto path = std::filesystem::path(calibration_dir_) / filename;
  std::ofstream os(path, std::ios::out | std::ios::trunc);
  if (!os.is_open()) {
    throw std::runtime_error("failed to open calibration file: " + path.string());
  }

  os << "header_frame_id: " << yamlDoubleQuoted(msg.header.frame_id) << "\n";
  os << "width: " << msg.width << "\n";
  os << "height: " << msg.height << "\n";
  os << "distortion_model: " << yamlDoubleQuoted(msg.distortion_model) << "\n";
  writeScalarArray(os, "d", msg.d);
  writeScalarArray(os, "k", msg.k);
  writeScalarArray(os, "r", msg.r);
  writeScalarArray(os, "p", msg.p);
  os << "binning_x: " << msg.binning_x << "\n";
  os << "binning_y: " << msg.binning_y << "\n";
  os << "roi:\n";
  os << "  x_offset: " << msg.roi.x_offset << "\n";
  os << "  y_offset: " << msg.roi.y_offset << "\n";
  os << "  height: " << msg.roi.height << "\n";
  os << "  width: " << msg.roi.width << "\n";
  os << "  do_rectify: " << (msg.roi.do_rectify ? "true" : "false") << "\n";
}

void SessionWriter::writeBodyToCameraExtrinsics(
  const std::string &body_frame_id,
  const std::string &camera_frame_id,
  const geometry_msgs::msg::TransformStamped &transform,
  const std::string &filename)
{
  const auto path = std::filesystem::path(calibration_dir_) / filename;
  std::ofstream os(path, std::ios::out | std::ios::trunc);
  if (!os.is_open()) {
    throw std::runtime_error("failed to open extrinsics file: " + path.string());
  }

  os << "body_frame_id: " << yamlDoubleQuoted(body_frame_id) << "\n";
  os << "camera_frame_id: " << yamlDoubleQuoted(camera_frame_id) << "\n";
  os << "body_T_camera:\n";
  os << "  position: ["
     << transform.transform.translation.x << ", "
     << transform.transform.translation.y << ", "
     << transform.transform.translation.z << "]\n";
  os << "  orientation_xyzw: ["
     << transform.transform.rotation.x << ", "
     << transform.transform.rotation.y << ", "
     << transform.transform.rotation.z << ", "
     << transform.transform.rotation.w << "]\n";
}

void SessionWriter::writeRecord(const CompletedRecord &record)
{
  const auto prefix = formatKfId(record.pending.keyframe.kf_id);
  const auto rgb_path = writeImage_(
    record.pending.rgb_msg,
    std::filesystem::path(rgb_dir_) / (prefix + imageExtensionFor_(*record.pending.rgb_msg, false)));

  std::filesystem::path depth_path;
  if (record.pending.have_depth && record.pending.depth_msg) {
    depth_path = writeImage_(
      record.pending.depth_msg,
      std::filesystem::path(depth_dir_) /
        (prefix + imageExtensionFor_(*record.pending.depth_msg, true)));
  }

  std::filesystem::path tags_path;
  if (record.pending.have_tags) {
    tags_path = writeTags_(record.pending, std::filesystem::path(tags_dir_) / (prefix + ".yaml"));
  }

  const auto keyframe_meta_path = writeKeyframeMetadata_(
    record.pending,
    rgb_path,
    depth_path,
    tags_path,
    std::filesystem::path(keyframes_dir_) / (prefix + ".yaml"));

  appendManifestLine_(record.pending, rgb_path, depth_path, tags_path, keyframe_meta_path);
}

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
  const auto manifest_path = std::filesystem::path(session_dir_) / "keyframe_manifest.csv";
  manifest_.open(manifest_path, std::ios::out | std::ios::trunc);
  if (!manifest_.is_open()) {
    throw std::runtime_error("failed to open manifest: " + manifest_path.string());
  }

  manifest_
    << "kf_id,keyframe_stamp_ns,rgb_stamp_ns,depth_stamp_ns,tags_stamp_ns,"
    << "rgb_path,depth_path,tags_path,keyframe_meta_path,"
    << "frontend_px,frontend_py,frontend_pz,frontend_qx,frontend_qy,frontend_qz,frontend_qw,"
    << "opt_body_px,opt_body_py,opt_body_pz,opt_body_qx,opt_body_qy,opt_body_qz,opt_body_qw,"
    << "track_count,tag_count\n";
  manifest_.flush();
}

void SessionWriter::writeSessionMetadata_()
{
  const auto metadata_path = std::filesystem::path(session_dir_) / "session_metadata.yaml";
  std::ofstream os(metadata_path, std::ios::out | std::ios::trunc);
  if (!os.is_open()) {
    throw std::runtime_error("failed to open metadata file: " + metadata_path.string());
  }

  os << "session_name: " << yamlDoubleQuoted(config_.session_name) << "\n";
  os << "created_at_utc: " << yamlDoubleQuoted(nowUtcString()) << "\n";
  os << "dataset_version: 1\n";
  os << "frames:\n";
  os << "  body: " << yamlDoubleQuoted(config_.body_frame_id) << "\n";
  os << "topics:\n";
  os << "  rgb_image: " << yamlDoubleQuoted(config_.rgb_image_topic) << "\n";
  os << "  rgb_camera_info: " << yamlDoubleQuoted(config_.rgb_camera_info_topic) << "\n";
  os << "  depth_image: " << yamlDoubleQuoted(config_.depth_image_topic) << "\n";
  os << "  depth_camera_info: " << yamlDoubleQuoted(config_.depth_camera_info_topic) << "\n";
  os << "  keyframe: " << yamlDoubleQuoted(config_.keyframe_topic) << "\n";
  os << "  optimization_result: " << yamlDoubleQuoted(config_.optimization_result_topic) << "\n";
  os << "  tag: " << yamlDoubleQuoted(config_.tag_topic) << "\n";
  os << "matching:\n";
  os << "  rgb_tolerance_ns: " << config_.rgb_match_tolerance_ns << "\n";
  os << "  depth_tolerance_ns: " << config_.depth_match_tolerance_ns << "\n";
  os << "  tag_tolerance_ns: " << config_.tag_match_tolerance_ns << "\n";
  os << "  tag_aggregation_window_ns: " << config_.tag_aggregation_window_ns << "\n";
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

std::filesystem::path SessionWriter::writeImage_(
  const ImageMsg::ConstSharedPtr &msg,
  const std::filesystem::path &path) const
{
  auto cv_image = cv_bridge::toCvCopy(msg, msg->encoding);
  if (!cv_image) {
    throw std::runtime_error("cv_bridge returned null image for " + path.string());
  }
  if (!cv::imwrite(path.string(), cv_image->image)) {
    throw std::runtime_error("failed to write image to " + path.string());
  }
  return path;
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
  os << "header_frame_id: " << yamlDoubleQuoted(pending.keyframe.header.frame_id) << "\n";
  os << "t_start: " << pending.keyframe.t_start << "\n";
  os << "t_end: " << pending.keyframe.t_end << "\n";
  os << "kf_reason_mask: " << pending.keyframe.kf_reason_mask << "\n";
  os << "has_vo_between: " << ((pending.keyframe.has_vo_between != 0U) ? "true" : "false") << "\n";
  os << "has_imu: " << static_cast<int>(pending.keyframe.has_imu) << "\n";
  os << "pim_bytes_hex: " << yamlDoubleQuoted(toHex(pending.keyframe.pim_bytes)) << "\n";
  writePoseYaml(os, "frontend_pose_ob", pending.keyframe.pose_odom_body);
  if (pending.keyframe.has_vo_between != 0U) {
    writePoseYaml(os, "between_pose_prev_curr_body", pending.keyframe.between_pose_prev_curr);
  }
  writePoseYaml(os, "optimized_pose_wb", pending.opt_result.pose_wb_opt);
  os << "optimization:\n";
  os << "  t_s: " << pending.opt_result.t_s << "\n";
  os << "  accel_bias: ["
     << pending.opt_result.accel_bias.x << ", "
     << pending.opt_result.accel_bias.y << ", "
     << pending.opt_result.accel_bias.z << "]\n";
  os << "  gyro_bias: ["
     << pending.opt_result.gyro_bias.x << ", "
     << pending.opt_result.gyro_bias.y << ", "
     << pending.opt_result.gyro_bias.z << "]\n";

  os << "associated_files:\n";
  os << "  rgb_path: " << yamlDoubleQuoted(relativeTo(rgb_path, session_dir_).generic_string()) << "\n";
  os << "  depth_path: "
     << yamlDoubleQuoted(depth_path.empty() ? std::string() : relativeTo(depth_path, session_dir_).generic_string())
     << "\n";
  os << "  tags_path: "
     << yamlDoubleQuoted(tags_path.empty() ? std::string() : relativeTo(tags_path, session_dir_).generic_string())
     << "\n";
  os << "associated_stamps_ns:\n";
  os << "  rgb: " << pending.rgb_stamp_ns << "\n";
  os << "  depth: " << pending.depth_stamp_ns << "\n";
  os << "  tags: " << pending.tags_stamp_ns << "\n";
  os << "tag_pose_count: " << pending.tag_poses.size() << "\n";

  os << "tracks:\n";
  writeScalarArray(os, "  track_ids", pending.keyframe.track_ids);
  writeScalarArray(os, "  u_l", pending.keyframe.u_l);
  writeScalarArray(os, "  v_l", pending.keyframe.v_l);
  writeScalarArray(os, "  u_r", pending.keyframe.u_r);
  writeScalarArray(os, "  v_r", pending.keyframe.v_r);
  writeScalarArray(os, "  has_right", pending.keyframe.has_right);

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
     << yamlDoubleQuoted(pending.tags_msg ? pending.tags_msg->header.frame_id : std::string())
     << "\n";
  os << "source_message_count: " << pending.tag_window_msgs.size() << "\n";
  os << "detections:\n";
  for (const auto &tag_pose : pending.tag_poses) {
    os << "  - family: " << yamlDoubleQuoted(tag_pose.family) << "\n";
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
    os << "      detection_frame_id: " << yamlDoubleQuoted(tag_pose.detection_frame_id) << "\n";
    os << "      parent_frame_id: " << yamlDoubleQuoted(tag_pose.parent_frame_id) << "\n";
    os << "      child_frame_id: " << yamlDoubleQuoted(tag_pose.child_frame_id) << "\n";
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
      os << "      lookup_error: " << yamlDoubleQuoted(tag_pose.lookup_error) << "\n";
    }
  }
#else
  os << "tag_message_type: \"unavailable_at_build_time\"\n";
  os << "header_stamp_ns: " << pending.tags_stamp_ns << "\n";
  os << "detections: []\n";
#endif
  return path;
}

void SessionWriter::appendManifestLine_(
  const PendingKeyframe &pending,
  const std::filesystem::path &rgb_path,
  const std::filesystem::path &depth_path,
  const std::filesystem::path &tags_path,
  const std::filesystem::path &keyframe_meta_path)
{
  const auto rgb_rel = relativeTo(rgb_path, session_dir_).generic_string();
  const auto depth_rel = depth_path.empty() ? std::string() : relativeTo(depth_path, session_dir_).generic_string();
  const auto tags_rel = tags_path.empty() ? std::string() : relativeTo(tags_path, session_dir_).generic_string();
  const auto meta_rel = relativeTo(keyframe_meta_path, session_dir_).generic_string();

  const size_t tag_count = pending.tag_poses.size();

  manifest_
    << pending.keyframe.kf_id << ","
    << pending.keyframe_stamp_ns << ","
    << pending.rgb_stamp_ns << ","
    << pending.depth_stamp_ns << ","
    << pending.tags_stamp_ns << ","
    << csvEscape(rgb_rel) << ","
    << csvEscape(depth_rel) << ","
    << csvEscape(tags_rel) << ","
    << csvEscape(meta_rel) << ","
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
  manifest_.flush();
}

std::string SessionWriter::imageExtensionFor_(const ImageMsg &msg, bool is_depth) const
{
  if (!is_depth) {
    return ".png";
  }

  if (msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
      msg.encoding == sensor_msgs::image_encodings::TYPE_32FC2 ||
      msg.encoding == sensor_msgs::image_encodings::TYPE_32FC3 ||
      msg.encoding == sensor_msgs::image_encodings::TYPE_32FC4) {
    return ".tiff";
  }
  return ".png";
}

}  // namespace online_mapping_logger
