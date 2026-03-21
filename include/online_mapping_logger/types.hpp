#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/optimization_result.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#endif

namespace online_mapping_logger
{

using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using KeyframeMsg = visual_inertial::msg::Keyframe;
using OptimizationResultMsg = visual_inertial::msg::OptimizationResult;

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
using TagArrayMsg = apriltag_msgs::msg::AprilTagDetectionArray;
using TagArrayMsgConstSharedPtr = TagArrayMsg::ConstSharedPtr;
#else
struct TagArrayMsg
{
};
using TagArrayMsgConstSharedPtr = std::shared_ptr<const TagArrayMsg>;
#endif

inline int64_t stampToNs(const builtin_interfaces::msg::Time &stamp)
{
  return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

struct LoggerConfig
{
  std::string output_root;
  std::string session_name;
  bool overwrite_existing_session{false};
  std::string body_frame_id{"body"};

  std::string rgb_image_topic;
  std::string rgb_camera_info_topic;
  std::string depth_image_topic;
  std::string depth_camera_info_topic;
  std::string keyframe_topic;
  std::string optimization_result_topic;
  std::string tag_topic;
  double tag_tf_lookup_timeout_ms{50.0};
  std::vector<int64_t> tag_frame_ids;
  std::vector<std::string> tag_frame_names;

  int64_t rgb_match_tolerance_ns{0};
  int64_t depth_match_tolerance_ns{0};
  int64_t tag_match_tolerance_ns{0};
  int64_t buffer_duration_ns{0};
  int64_t pending_timeout_ns{0};
  int maintenance_period_ms{250};

  bool require_depth{true};
  bool require_tags{true};

  std::string tag_prior_source_path;

  bool depthStreamEnabled() const
  {
    return !depth_image_topic.empty();
  }

  bool tagStreamEnabled() const
  {
    return !tag_topic.empty();
  }
};

struct PendingKeyframe
{
  KeyframeMsg keyframe;
  int64_t keyframe_stamp_ns{0};
  int64_t created_at_ns{0};

  bool have_rgb{false};
  ImageMsg::ConstSharedPtr rgb_msg;
  int64_t rgb_stamp_ns{0};

  bool have_depth{false};
  ImageMsg::ConstSharedPtr depth_msg;
  int64_t depth_stamp_ns{0};

  bool have_opt_result{false};
  OptimizationResultMsg opt_result;

  bool have_tags{false};
  TagArrayMsgConstSharedPtr tags_msg;
  int64_t tags_stamp_ns{0};
  struct LoggedTagPose
  {
    std::string family;
    int32_t id{0};
    std::string detection_frame_id;
    std::string parent_frame_id;
    std::string child_frame_id;
    int64_t lookup_stamp_ns{0};
    bool pose_available{false};
    geometry_msgs::msg::TransformStamped transform;
    std::string lookup_error;
  };
  std::vector<LoggedTagPose> tag_poses;
};

struct CompletedRecord
{
  PendingKeyframe pending;
};

struct TimedOutKeyframe
{
  uint64_t kf_id{0};
  int64_t stamp_ns{0};
  std::vector<std::string> missing_fields;
};

struct AssemblerUpdate
{
  std::vector<CompletedRecord> completed_records;
  std::vector<TimedOutKeyframe> timed_out_keyframes;
};

}  // namespace online_mapping_logger
