#include "online_mapping_logger/keyframe_assembler.hpp"
#include "online_mapping_logger/session_writer.hpp"
#include "online_mapping_logger/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#endif

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace online_mapping_logger
{

namespace
{

int64_t msToNs(double ms)
{
  return static_cast<int64_t>(ms * 1000000.0);
}

int64_t secondsToNs(double seconds)
{
  return static_cast<int64_t>(seconds * 1000000000.0);
}

}  // namespace

class OnlineMappingLoggerNode final : public rclcpp::Node
{
public:
  OnlineMappingLoggerNode()
  : Node("online_mapping_logger")
  {
    config_ = loadConfig_();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    writer_ = std::make_unique<SessionWriter>(config_);
    assembler_ = std::make_unique<KeyframeAssembler>(config_);

    createSubscriptions_();

    maintenance_timer_ = create_wall_timer(
      std::chrono::milliseconds(config_.maintenance_period_ms),
      std::bind(&OnlineMappingLoggerNode::maintenanceTick_, this));

    RCLCPP_INFO(
      get_logger(),
      "online_mapping_logger writing session to '%s'",
      writer_->sessionDir().c_str());
  }

private:
  LoggerConfig loadConfig_()
  {
    LoggerConfig config;
    config.output_root = declare_parameter<std::string>("output_root", "/tmp/online_mapping_sessions");
    config.session_name = declare_parameter<std::string>("session_name", "");
    config.overwrite_existing_session =
      declare_parameter<bool>("overwrite_existing_session", false);
    config.body_frame_id = declare_parameter<std::string>("body_frame_id", "body");

    config.rgb_image_topic = declare_parameter<std::string>(
      "rgb_image_topic", "/camera0/realsense_splitter_node/output/infra_1");
    config.rgb_camera_info_topic =
      declare_parameter<std::string>("rgb_camera_info_topic", "/camera0/infra1/camera_info");
    config.depth_image_topic = declare_parameter<std::string>(
      "depth_image_topic", "/camera0/realsense_splitter_node/output/depth");
    config.depth_camera_info_topic =
      declare_parameter<std::string>("depth_camera_info_topic", "/camera0/depth/camera_info");
    config.keyframe_topic = declare_parameter<std::string>("keyframe_topic", "/keyframes");
    config.optimization_result_topic =
      declare_parameter<std::string>("optimization_result_topic", "/optimization_result");
    config.tag_topic = declare_parameter<std::string>("tag_topic", "/detections");
    config.tag_tf_lookup_timeout_ms =
      declare_parameter<double>("tag_tf_lookup_timeout_ms", 50.0);
    config.tag_frame_ids =
      declare_parameter<std::vector<int64_t>>("tag_frame_ids", std::vector<int64_t>{});
    config.tag_frame_names =
      declare_parameter<std::vector<std::string>>("tag_frame_names", std::vector<std::string>{});

    config.rgb_match_tolerance_ns =
      msToNs(declare_parameter<double>("rgb_match_tolerance_ms", 50.0));
    config.depth_match_tolerance_ns =
      msToNs(declare_parameter<double>("depth_match_tolerance_ms", 25.0));
    config.tag_match_tolerance_ns =
      msToNs(declare_parameter<double>("tag_match_tolerance_ms", 25.0));
    config.buffer_duration_ns =
      secondsToNs(declare_parameter<double>("buffer_duration_s", 5.0));
    config.pending_timeout_ns =
      secondsToNs(declare_parameter<double>("pending_timeout_s", 2.0));
    config.maintenance_period_ms =
      declare_parameter<int>("maintenance_period_ms", 250);

    config.require_depth = declare_parameter<bool>("require_depth", true);
    config.require_tags = declare_parameter<bool>("require_tags", false);
    config.tag_prior_source_path =
      declare_parameter<std::string>("tag_prior_source_path", "");

    if (config.session_name.empty()) {
      config.session_name = defaultSessionName();
    }
    config.session_name = sanitizeFilenameComponent(config.session_name);

    if (!config.depthStreamEnabled()) {
      config.require_depth = false;
    }
    if (!config.tagStreamEnabled()) {
      config.require_tags = false;
    }

    if (!config.tag_frame_names.empty() &&
        config.tag_frame_ids.size() != config.tag_frame_names.size()) {
      throw std::runtime_error(
        "tag_frame_ids and tag_frame_names must have the same length");
    }

    tag_frame_overrides_.clear();
    for (size_t i = 0; i < config.tag_frame_ids.size(); ++i) {
      tag_frame_overrides_[static_cast<int32_t>(config.tag_frame_ids[i])] =
        config.tag_frame_names[i];
    }

#ifndef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
    if (config.require_tags) {
      RCLCPP_WARN(
        get_logger(),
        "Tag logging requested but this build has no AprilTag message support. "
        "Tag matching will be disabled.");
      config.require_tags = false;
    }
#endif

    return config;
  }

  void createSubscriptions_()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();

    rgb_sub_ = create_subscription<ImageMsg>(
      config_.rgb_image_topic,
      sensor_qos,
      [this](ImageMsg::ConstSharedPtr msg) {
        handleAssemblerUpdate_(assembler_->addRgb(msg));
      });

    if (config_.depthStreamEnabled()) {
      depth_sub_ = create_subscription<ImageMsg>(
        config_.depth_image_topic,
        sensor_qos,
        [this](ImageMsg::ConstSharedPtr msg) {
          handleAssemblerUpdate_(assembler_->addDepth(msg));
        });
    }

    if (!config_.rgb_camera_info_topic.empty()) {
      rgb_info_sub_ = create_subscription<CameraInfoMsg>(
        config_.rgb_camera_info_topic,
        sensor_qos,
        [this](CameraInfoMsg::ConstSharedPtr msg) {
          writer_->writeCameraInfo(*msg, "rgb_camera_info.yaml");
          maybeWriteCameraExtrinsics_(*msg, "body_to_rgb_camera.yaml", rgb_extrinsics_written_);
        });
    }

    if (!config_.depth_camera_info_topic.empty()) {
      depth_info_sub_ = create_subscription<CameraInfoMsg>(
        config_.depth_camera_info_topic,
        sensor_qos,
        [this](CameraInfoMsg::ConstSharedPtr msg) {
          writer_->writeCameraInfo(*msg, "depth_camera_info.yaml");
          maybeWriteCameraExtrinsics_(*msg, "body_to_depth_camera.yaml", depth_extrinsics_written_);
        });
    }

    keyframe_sub_ = create_subscription<KeyframeMsg>(
      config_.keyframe_topic,
      reliable_qos,
      [this](KeyframeMsg::ConstSharedPtr msg) {
        handleAssemblerUpdate_(assembler_->addKeyframe(msg, now().nanoseconds()));
      });

    optimization_sub_ = create_subscription<OptimizationResultMsg>(
      config_.optimization_result_topic,
      reliable_qos,
      [this](OptimizationResultMsg::ConstSharedPtr msg) {
        handleAssemblerUpdate_(assembler_->addOptimizationResult(msg));
      });

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
    if (config_.tagStreamEnabled()) {
      tag_sub_ = create_subscription<TagArrayMsg>(
        config_.tag_topic,
        sensor_qos,
        [this](TagArrayMsg::ConstSharedPtr msg) {
          handleAssemblerUpdate_(assembler_->addTags(msg));
        });
    }
#endif
  }

  void maintenanceTick_()
  {
    handleAssemblerUpdate_(assembler_->maintenance(now().nanoseconds()));
  }

  void handleAssemblerUpdate_(const AssemblerUpdate &update)
  {
    for (const auto &record : update.completed_records) {
      auto record_with_tag_poses = record;
      resolveTagPoses_(record_with_tag_poses.pending);
      writer_->writeRecord(record_with_tag_poses);
      size_t resolved_tag_pose_count = 0;
      for (const auto &tag_pose : record_with_tag_poses.pending.tag_poses) {
        if (tag_pose.pose_available) {
          ++resolved_tag_pose_count;
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "Saved keyframe dataset kf_id=%lu stamp_ns=%lld tags=%zu resolved_tag_poses=%zu",
        static_cast<unsigned long>(record_with_tag_poses.pending.keyframe.kf_id),
        static_cast<long long>(record_with_tag_poses.pending.keyframe_stamp_ns),
        record_with_tag_poses.pending.tag_poses.size(),
        resolved_tag_pose_count);
    }

    for (const auto &timed_out : update.timed_out_keyframes) {
      std::string missing;
      for (size_t i = 0; i < timed_out.missing_fields.size(); ++i) {
        if (i > 0) {
          missing += ", ";
        }
        missing += timed_out.missing_fields[i];
      }

      RCLCPP_WARN(
        get_logger(),
        "Dropping stale pending keyframe kf_id=%lu stamp_ns=%lld missing=[%s]",
        static_cast<unsigned long>(timed_out.kf_id),
        static_cast<long long>(timed_out.stamp_ns),
        missing.c_str());
    }
  }

  void resolveTagPoses_(PendingKeyframe &pending)
  {
#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
    pending.tag_poses.clear();
    if (!pending.have_tags || !tf_buffer_) {
      return;
    }

    const auto lookup_timeout =
      rclcpp::Duration::from_seconds(config_.tag_tf_lookup_timeout_ms / 1000.0);

    if (!pending.tags_msg) {
      return;
    }

    const auto lookup_time = rclcpp::Time(pending.tags_msg->header.stamp);
    pending.tag_poses.reserve(pending.tags_msg->detections.size());
    for (const auto &detection : pending.tags_msg->detections) {
      PendingKeyframe::LoggedTagPose tag_pose;
      tag_pose.family = detection.family;
      tag_pose.id = detection.id;
      tag_pose.hamming = detection.hamming;
      tag_pose.goodness = detection.goodness;
      tag_pose.decision_margin = detection.decision_margin;
      tag_pose.centre = {detection.centre.x, detection.centre.y};
      tag_pose.homography.assign(detection.homography.begin(), detection.homography.end());
      for (const auto &corner : detection.corners) {
        tag_pose.corners.push_back({corner.x, corner.y});
      }
      tag_pose.detection_frame_id = pending.tags_msg->header.frame_id;
      tag_pose.parent_frame_id = config_.body_frame_id;
      const auto override_it = tag_frame_overrides_.find(detection.id);
      if (override_it != tag_frame_overrides_.end()) {
        tag_pose.child_frame_id = override_it->second;
      } else {
        tag_pose.child_frame_id = detection.family + ":" + std::to_string(detection.id);
      }
      tag_pose.lookup_stamp_ns = pending.tags_stamp_ns;
      tag_pose.sample_count = 1;

      try {
        tag_pose.transform = tf_buffer_->lookupTransform(
          tag_pose.parent_frame_id,
          tag_pose.child_frame_id,
          lookup_time,
          lookup_timeout);
        tag_pose.pose_available = true;
        tag_pose.resolved_sample_count = 1;
      } catch (const tf2::TransformException &ex) {
        tag_pose.lookup_error = ex.what();
      }

      pending.tag_poses.push_back(std::move(tag_pose));
    }
#else
    (void)pending;
#endif
  }

  void maybeWriteCameraExtrinsics_(
    const CameraInfoMsg &msg,
    const std::string &filename,
    bool &already_written)
  {
    if (already_written || !tf_buffer_) {
      return;
    }
    if (msg.header.frame_id.empty()) {
      return;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        config_.body_frame_id,
        msg.header.frame_id,
        tf2::TimePointZero);
      writer_->writeBodyToCameraExtrinsics(
        config_.body_frame_id, msg.header.frame_id, transform, filename);
      already_written = true;
      RCLCPP_INFO(
        get_logger(),
        "Wrote body->camera extrinsics '%s' for %s <- %s",
        filename.c_str(),
        config_.body_frame_id.c_str(),
        msg.header.frame_id.c_str());
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF %s <- %s before writing %s: %s",
        config_.body_frame_id.c_str(),
        msg.header.frame_id.c_str(),
        filename.c_str(),
        ex.what());
    }
  }

  LoggerConfig config_;
  std::unordered_map<int32_t, std::string> tag_frame_overrides_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<SessionWriter> writer_;
  std::unique_ptr<KeyframeAssembler> assembler_;

  rclcpp::Subscription<ImageMsg>::SharedPtr rgb_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr rgb_info_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<KeyframeMsg>::SharedPtr keyframe_sub_;
  rclcpp::Subscription<OptimizationResultMsg>::SharedPtr optimization_sub_;
#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
  rclcpp::Subscription<TagArrayMsg>::SharedPtr tag_sub_;
#endif
  rclcpp::TimerBase::SharedPtr maintenance_timer_;
  bool rgb_extrinsics_written_{false};
  bool depth_extrinsics_written_{false};
};

}  // namespace online_mapping_logger

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<online_mapping_logger::OnlineMappingLoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
