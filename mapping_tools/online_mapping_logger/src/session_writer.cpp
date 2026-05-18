#include "online_mapping_logger/session_writer.hpp"

#include "session_writer_detail.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <fstream>
#include <stdexcept>

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

  os << "header_frame_id: " << detail::yamlDoubleQuoted(msg.header.frame_id) << "\n";
  os << "width: " << msg.width << "\n";
  os << "height: " << msg.height << "\n";
  os << "distortion_model: " << detail::yamlDoubleQuoted(msg.distortion_model) << "\n";
  detail::writeScalarArray(os, "d", msg.d);
  detail::writeScalarArray(os, "k", msg.k);
  detail::writeScalarArray(os, "r", msg.r);
  detail::writeScalarArray(os, "p", msg.p);
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

  os << "body_frame_id: " << detail::yamlDoubleQuoted(body_frame_id) << "\n";
  os << "camera_frame_id: " << detail::yamlDoubleQuoted(camera_frame_id) << "\n";
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
