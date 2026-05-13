#include <rclcpp/rclcpp.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/localization_command.hpp>
#include <visual_inertial/msg/localization_feedback.hpp>
#include <visual_inertial/msg/localization_pose_prior.hpp>

#include <visual_inertial/transport/common_transport.hpp>
#include <visual_inertial/transport/localization_transport.hpp>
#include <visual_inertial_localization/localization.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>
#include <functional>
#include <vector>

namespace
{

struct LocalizationNodeRuntimeConfig
{
    std::string keyframe_topic{"keyframes"};
    std::string body_frame_id{"body"};
    std::string odom_frame_id{"odom"};
    std::string localization_command_topic{"localization_command"};
    std::string localization_feedback_topic{"localization_feedback"};
};

LocalizationNodeRuntimeConfig declareRuntimeConfig(rclcpp::Node &node)
{
    LocalizationNodeRuntimeConfig cfg;
    cfg.keyframe_topic =
        node.declare_parameter<std::string>("keyframe_topic", cfg.keyframe_topic);
    cfg.body_frame_id =
        node.declare_parameter<std::string>("body_frame_id", cfg.body_frame_id);
    cfg.odom_frame_id =
        node.declare_parameter<std::string>("odom_frame_id", cfg.odom_frame_id);
    cfg.localization_command_topic = node.declare_parameter<std::string>(
        "localization_command_topic", cfg.localization_command_topic);
    cfg.localization_feedback_topic = node.declare_parameter<std::string>(
        "localization_feedback_topic", cfg.localization_feedback_topic);
    return cfg;
}

visual_inertial_localization::LocalizationConfig declareLocalizationConfig(rclcpp::Node &node)
{
    auto cfg = visual_inertial_localization::LocalizationConfig{};

    cfg.tag_map_path =
        node.declare_parameter<std::string>("localization_tag_map_path", "");
    cfg.tag_topic =
        node.declare_parameter<std::string>("tag_topic", cfg.tag_topic);
    cfg.tag_tf_lookup_timeout_ms =
        node.declare_parameter<double>("tag_tf_lookup_timeout_ms", cfg.tag_tf_lookup_timeout_ms);
    cfg.tag_max_age_s =
        node.declare_parameter<double>("localization_tag_max_age_s", cfg.tag_max_age_s);
    cfg.tag_buffer_age_s =
        node.declare_parameter<double>("tag_buffer_age_s", cfg.tag_buffer_age_s);
    cfg.max_tag_hamming =
        node.declare_parameter<int>("max_tag_hamming", cfg.max_tag_hamming);
    cfg.min_tag_decision_margin =
        node.declare_parameter<double>("min_tag_decision_margin", cfg.min_tag_decision_margin);
    cfg.max_tag_range_m =
        node.declare_parameter<double>("max_tag_range_m", cfg.max_tag_range_m);
    cfg.max_tag_oblique_angle_deg =
        node.declare_parameter<double>("max_tag_oblique_angle_deg", cfg.max_tag_oblique_angle_deg);
    cfg.pose_prior_rot_sigma_rad =
        node.declare_parameter<double>("localization_pose_prior_rot_sigma_rad", cfg.pose_prior_rot_sigma_rad);
    cfg.pose_prior_trans_sigma_m =
        node.declare_parameter<double>("localization_pose_prior_trans_sigma_m", cfg.pose_prior_trans_sigma_m);
    cfg.pose_prior_huber_k =
        node.declare_parameter<double>("localization_pose_prior_huber_k", cfg.pose_prior_huber_k);
    cfg.cluster_translation_m =
        node.declare_parameter<double>("localization_cluster_translation_m", cfg.cluster_translation_m);
    cfg.cluster_rotation_deg =
        node.declare_parameter<double>("localization_cluster_rotation_deg", cfg.cluster_rotation_deg);
    cfg.stable_hypothesis_age_s =
        node.declare_parameter<double>("localization_stable_hypothesis_age_s", cfg.stable_hypothesis_age_s);
    cfg.stable_min_frames = static_cast<size_t>(
        node.declare_parameter<int>("localization_stable_min_frames", static_cast<int>(cfg.stable_min_frames)));
    cfg.relocalization_min_history_frames = static_cast<size_t>(
        node.declare_parameter<int>(
            "localization_relocalization_min_history_frames",
            static_cast<int>(cfg.relocalization_min_history_frames)));
    cfg.stable_translation_m =
        node.declare_parameter<double>("localization_stable_translation_m", cfg.stable_translation_m);
    cfg.stable_rotation_deg =
        node.declare_parameter<double>("localization_stable_rotation_deg", cfg.stable_rotation_deg);
    cfg.relocalize_translation_m =
        node.declare_parameter<double>("localization_relocalize_translation_m", cfg.relocalize_translation_m);
    cfg.relocalize_rotation_deg =
        node.declare_parameter<double>("localization_relocalize_rotation_deg", cfg.relocalize_rotation_deg);
    cfg.tracking_deadband_translation_m =
        node.declare_parameter<double>(
            "localization_tracking_deadband_translation_m",
            cfg.tracking_deadband_translation_m);
    cfg.tracking_deadband_rotation_deg =
        node.declare_parameter<double>(
            "localization_tracking_deadband_rotation_deg",
            cfg.tracking_deadband_rotation_deg);

    const auto tag_frame_ids = node.declare_parameter<std::vector<int64_t>>(
        "tag_frame_ids", std::vector<int64_t>{});
    const auto tag_frame_names = node.declare_parameter<std::vector<std::string>>(
        "tag_frame_names", std::vector<std::string>{});

    if (tag_frame_ids.size() != tag_frame_names.size())
    {
        RCLCPP_WARN(
            node.get_logger(),
            "tag_frame_ids/tag_frame_names size mismatch (%zu vs %zu); ignoring overrides",
            tag_frame_ids.size(),
            tag_frame_names.size());
        return cfg;
    }

    for (size_t i = 0; i < tag_frame_ids.size(); ++i)
    {
        cfg.tag_frame_overrides.emplace(
            static_cast<int>(tag_frame_ids[i]),
            tag_frame_names[i]);
    }

    return cfg;
}

} // namespace

class LocalizationNode final : public rclcpp::Node
{
public:
    LocalizationNode()
        : rclcpp::Node("visual_inertial_localization")
    {
        runtime_cfg_ = declareRuntimeConfig(*this);
        auto loc_cfg = declareLocalizationConfig(*this);
        localization_ = std::make_unique<visual_inertial_localization::LocalizationModule>(
            std::move(loc_cfg));

        const auto report = localization_->loadTagMap();
        if (localization_->config().tag_map_path.empty())
        {
            RCLCPP_INFO(
                get_logger(),
                "No tag map path configured; localization node will publish odometry-only commands");
        }
        else if (report.ok)
        {
            RCLCPP_INFO(
                get_logger(),
                "Localization tag map loaded from %s with %zu tags and %zu frame overrides",
                localization_->config().tag_map_path.c_str(),
                report.mapped_tag_count,
                report.frame_override_count);
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Localization tag map could not be loaded from %s: %s",
                localization_->config().tag_map_path.c_str(),
                report.message.c_str());
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto command_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
        command_pub_ = create_publisher<visual_inertial::msg::LocalizationCommand>(
            runtime_cfg_.localization_command_topic, command_qos);

        auto keyframe_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
        keyframe_sub_ = create_subscription<visual_inertial::msg::Keyframe>(
            runtime_cfg_.keyframe_topic,
            keyframe_qos,
            std::bind(&LocalizationNode::onKeyframe_, this, std::placeholders::_1));

        auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
        feedback_sub_ = create_subscription<visual_inertial::msg::LocalizationFeedback>(
            runtime_cfg_.localization_feedback_topic,
            feedback_qos,
            std::bind(&LocalizationNode::onLocalizationFeedback_, this, std::placeholders::_1));

        auto tag_qos = rclcpp::SensorDataQoS();
        tag_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            localization_->config().tag_topic,
            tag_qos,
            std::bind(&LocalizationNode::onTagDetections_, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Localization node started. keyframes=%s tags=%s command=%s feedback=%s",
            runtime_cfg_.keyframe_topic.c_str(),
            localization_->config().tag_topic.c_str(),
            runtime_cfg_.localization_command_topic.c_str(),
            runtime_cfg_.localization_feedback_topic.c_str());
    }

private:
    void onKeyframe_(visual_inertial::msg::Keyframe::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        const auto decision = localization_->processKeyframe(
            rclcpp::Time(msg->header.stamp).nanoseconds(),
            visual_inertial::transport::poseMsgToIso(msg->pose_odom_body));
        command_pub_->publish(
            visual_inertial::transport::makeLocalizationCommandMsg(*msg, decision));
    }

    void onLocalizationFeedback_(visual_inertial::msg::LocalizationFeedback::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        localization_->updateMapOdomEstimate(
            visual_inertial::transport::poseMsgToIso(msg->pose_map_odom));
    }

    void onTagDetections_(apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if (!msg || !tf_buffer_)
        {
            return;
        }

        const auto report = localization_->ingestDetections(
            *msg,
            runtime_cfg_.body_frame_id,
            runtime_cfg_.odom_frame_id,
            *tf_buffer_);
        const auto stable_correction =
            localization_->estimateStableCorrection(rclcpp::Time(msg->header.stamp));
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Localization tag ingest: detections=%zu accepted=%zu skipped_unmapped=%zu "
            "skipped_hamming=%zu skipped_margin=%zu skipped_tf=%zu skipped_range=%zu skipped_oblique=%zu buffered=%zu stable_frames=%zu stable_support=%zu",
            report.total_detections,
            report.accepted,
            report.skipped_unmapped,
            report.skipped_hamming,
            report.skipped_margin,
            report.skipped_tf_lookup,
            report.skipped_range,
            report.skipped_oblique,
            report.buffered,
            stable_correction.has_value() ? stable_correction->frame_support : 0,
            stable_correction.has_value() ? stable_correction->support_count : 0);
    }

private:
    LocalizationNodeRuntimeConfig runtime_cfg_;
    std::unique_ptr<visual_inertial_localization::LocalizationModule> localization_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<visual_inertial::msg::LocalizationCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<visual_inertial::msg::Keyframe>::SharedPtr keyframe_sub_;
    rclcpp::Subscription<visual_inertial::msg::LocalizationFeedback>::SharedPtr feedback_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
