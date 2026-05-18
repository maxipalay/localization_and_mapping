#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visual_inertial/msg/localization_command.hpp>
#include <visual_inertial/msg/localization_feedback.hpp>
#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/optimization_result.hpp>
#include <visual_inertial/optimization_node_params.hpp>
#include <visual_inertial/transport/common_transport.hpp>
#include <visual_inertial/transport/localization_transport.hpp>
#include <visual_inertial/transport/optimization_transport.hpp>

#include <visual_inertial_common/types.hpp>
#include <visual_inertial_optimization/optimization.hpp>
#include <visual_inertial_optimization/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/core.hpp>

#include <Eigen/Geometry>

#include <mutex>
#include <condition_variable>
#include <deque>
#include <thread>
#include <atomic>
#include <optional>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <vector>

#include <visual_inertial/msg/imu_bias.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace
{
    static CameraRig makeStereoModel(
        const sensor_msgs::msg::CameraInfo &L,
        const sensor_msgs::msg::CameraInfo &R)
    {
        CameraRig rig;

        auto fetch = [](const sensor_msgs::msg::CameraInfo &info, int idx_p, int idx_k) -> double
        {
            return (info.p.size() == 12) ? info.p[idx_p] : info.k[idx_k];
        };

        rig.left.cam_id = 0;
        rig.left.K << fetch(L, 0, 0), 0.0, fetch(L, 2, 2),
            0.0, fetch(L, 5, 4), fetch(L, 6, 5),
            0.0, 0.0, 1.0;

        rig.right.cam_id = 1;
        rig.right.K << fetch(R, 0, 0), 0.0, fetch(R, 2, 2),
            0.0, fetch(R, 5, 4), fetch(R, 6, 5),
            0.0, 0.0, 1.0;

        const double fx_r = rig.right.K(0, 0);
        double Tx = 0.0;
        if (R.p.size() == 12 && std::abs(fx_r) > 1e-9)
        {
            Tx = -R.p[3] / fx_r;
        }
        rig.baseline = std::abs(Tx);

        return rig;
    }
} // namespace

class OptimizationNode final : public rclcpp::Node
{
public:
    OptimizationNode()
        : rclcpp::Node("visual_inertial_optimization")
    {
        param_handler_ = std::make_unique<visual_inertial::OptimizationNodeParamHandler>(*this);
        const auto &params = param_handler_->params();
        node_cfg_ = params.node;
        optimization_ = std::make_unique<OptimizationModule>(params.optimizer);

        initTf_();
        initPublishers_();
        initSubscriptions_();
        startWorker_();
        logStartup_();
    }

    ~OptimizationNode() override
    {
        stop_.store(true);
        q_cv_.notify_one();
        if (worker_.joinable())
            worker_.join();
    }

private:
    struct PreparedOptimizationUpdate
    {
        KeyframeEvent event;
        std::optional<Eigen::Isometry3d> between_meas;
        std::vector<AbsolutePosePrior> absolute_pose_priors;
        std::optional<Eigen::Isometry3d> T_WB_init_override;
        std::optional<Eigen::Isometry3d> T_WB_anchor_override;
        std::optional<visual_inertial::msg::LocalizationCommand> localization_command;
    };

    // Keyframe update preparation and localization decisions
    bool hasMalformedTrackPayload_(const visual_inertial::msg::Keyframe &msg) const
    {
        return (msg.u_l.size() != msg.track_ids.size()) ||
               (msg.v_l.size() != msg.track_ids.size()) ||
               (msg.u_r.size() != msg.track_ids.size()) ||
               (msg.v_r.size() != msg.track_ids.size()) ||
               (msg.has_right.size() != msg.track_ids.size());
    }

    void logMalformedTrackPayload_(const visual_inertial::msg::Keyframe &msg) const
    {
        RCLCPP_WARN(
            get_logger(),
            "Malformed keyframe payload kf_id=%lu: track_ids=%zu u_l=%zu v_l=%zu u_r=%zu v_r=%zu has_right=%zu has_imu=%d pim_bytes=%zu",
            static_cast<unsigned long>(msg.kf_id),
            msg.track_ids.size(),
            msg.u_l.size(),
            msg.v_l.size(),
            msg.u_r.size(),
            msg.v_r.size(),
            msg.has_right.size(),
            static_cast<int>(msg.has_imu),
            msg.pim_bytes.size());
    }

    std::optional<visual_inertial::msg::LocalizationCommand> waitForLocalizationCommand_(
        uint64_t kf_id)
    {
        std::unique_lock<std::mutex> lk(localization_cmd_mtx_);
        localization_cmd_cv_.wait_for(
            lk,
            std::chrono::duration<double, std::milli>(node_cfg_.localization_command_wait_ms),
            [&]
            { return stop_.load() || localization_cmd_by_kf_.count(kf_id) > 0; });

        auto it = localization_cmd_by_kf_.find(kf_id);
        if (it == localization_cmd_by_kf_.end())
        {
            return std::nullopt;
        }

        auto msg = it->second;
        localization_cmd_by_kf_.erase(it);
        for (auto stale = localization_cmd_by_kf_.begin(); stale != localization_cmd_by_kf_.end();)
        {
            if (stale->first < kf_id)
            {
                stale = localization_cmd_by_kf_.erase(stale);
            }
            else
            {
                ++stale;
            }
        }
        return msg;
    }

    PreparedOptimizationUpdate prepareOptimizationUpdate_(
        const visual_inertial::msg::Keyframe &msg)
    {
        PreparedOptimizationUpdate update;
        update.event = visual_inertial::transport::toKeyframeEvent(msg);

        if (update.event.has_vo_between)
        {
            update.between_meas = update.event.T_Bkm1_Bk;
        }

        if (!node_cfg_.localization_mode)
        {
            return update;
        }

        auto command = waitForLocalizationCommand_(msg.kf_id);
        if (!command.has_value())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Localization mode is enabled but no LocalizationCommand arrived for kf_id=%lu within %.1f ms; proceeding without localization inputs",
                static_cast<unsigned long>(msg.kf_id),
                node_cfg_.localization_command_wait_ms);
            return update;
        }

        update.localization_command = std::move(command);

        for (const auto &prior : update.localization_command->absolute_pose_priors)
        {
            update.absolute_pose_priors.push_back(
                visual_inertial::transport::toAbsolutePosePrior(prior));
        }
        if (update.localization_command->has_init_override)
        {
            update.T_WB_init_override =
                visual_inertial::transport::poseMsgToIso(
                    update.localization_command->pose_wb_init_override);
        }
        if (update.localization_command->has_anchor_override)
        {
            update.T_WB_anchor_override =
                visual_inertial::transport::poseMsgToIso(
                    update.localization_command->pose_wb_anchor_override);
        }

        return update;
    }

    void applyLocalizationDecision_(
        const PreparedOptimizationUpdate &update,
        OptimizationModule &optimization)
    {
        if (!update.localization_command.has_value())
        {
            return;
        }

        const auto &command = *update.localization_command;

        if (command.action == visual_inertial::msg::LocalizationCommand::ACTION_BOOTSTRAP)
        {
            optimization.reset();
            if (command.has_bootstrap_info)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Localization bootstrap established: support=%zu score=%.1f; switching from odometry-only to map-localized tracking",
                    static_cast<size_t>(command.bootstrap_support_count),
                    command.bootstrap_score);
            }
        }
        else if (command.waiting_for_bootstrap)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Localization mode has not seen a stable mapped tag yet; running in pure odometry until first bootstrap");
        }

        if (command.action == visual_inertial::msg::LocalizationCommand::ACTION_RELOCALIZE)
        {
            optimization.reset();
            RCLCPP_INFO(
                get_logger(),
                "Localization graph reset for relocalization at incoming kf_id=%lu",
                static_cast<unsigned long>(update.event.kf_id));

            if (command.has_relocalization_info)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Localization relocalization trigger: history_frames=%zu support=%zu map_odom_trans_error=%.3f m map_odom_rot_error=%.1f deg",
                    static_cast<size_t>(command.relocalization_frame_support),
                    static_cast<size_t>(command.relocalization_support_count),
                    command.relocalization_translation_error_m,
                    command.relocalization_rotation_error_rad * 180.0 / M_PI);
            }
        }
    }

    void logOptimizationUpdate_(
        const OptimizationResult &result,
        double optimize_ms) const
    {
        RCLCPP_INFO(
            get_logger(),
            "Optimization update kf_id=%lu took %.1f ms "
            "(window=%d stereo=%d imu=%d between=%d prior=%d "
            "vo_meas=%d vo_used=%d vo_skip=%d vo_q=%.3f vo_sigma=%.3f imu_only=%d)",
            static_cast<unsigned long>(result.kf_id),
            optimize_ms,
            result.stats.num_keyframes_in_window,
            result.stats.num_stereo_factors_added,
            result.stats.num_imu_factors_added,
            result.stats.num_between_factors_added,
            result.stats.num_prior_factors_added,
            result.stats.had_vo_between_measurement ? 1 : 0,
            result.stats.used_vo_between_factor ? 1 : 0,
            result.stats.skipped_vo_between_factor ? 1 : 0,
            result.stats.vo_between_quality,
            result.stats.vo_between_sigma_scale,
            result.stats.imu_only_update ? 1 : 0);
    }

    void publishImuBias_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result)
    {
        bias_pub_->publish(visual_inertial::transport::makeImuBiasMsg(msg, result));
    }

    void publishOptimizationResult_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result)
    {
        if (!optimization_result_pub_)
        {
            return;
        }

        optimization_result_pub_->publish(
            visual_inertial::transport::makeOptimizationResultMsg(
                msg, result, node_cfg_.map_frame_id));
    }

    Eigen::Isometry3d updateMapOdomTarget_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result)
    {
        const Eigen::Isometry3d T_odom_B =
            visual_inertial::transport::poseMsgToIso(msg.pose_odom_body);
        const Eigen::Isometry3d T_map_B = result.T_WB_opt;
        const Eigen::Isometry3d T_map_odom = T_map_B * T_odom_B.inverse();

        {
            std::lock_guard<std::mutex> lk(tf_mtx_);
            T_map_odom_target_ = T_map_odom;
            have_target_ = true;
        }

        return T_map_odom;
    }

    void publishLocalizationFeedback_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result)
    {
        if (localization_feedback_pub_)
        {
            localization_feedback_pub_->publish(
                visual_inertial::transport::makeLocalizationFeedbackMsg(msg, result));
        }
    }

    void handleOptimizationResult_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result,
        OptimizationModule &optimization,
        double optimize_ms)
    {
        logOptimizationUpdate_(result, optimize_ms);
        publishImuBias_(msg, result);
        publishOptimizationResult_(msg, result);
        publishLandmarks_(optimization);
        updateMapOdomTarget_(msg, result);
        publishLocalizationFeedback_(msg, result);
    }

    // Setup
    void initTf_()
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        using namespace std::chrono_literals;
        tf_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / node_cfg_.tf_pub_rate_hz),
            [this]()
            { this->publishMapOdomTf_(); });
    }

    void initPublishers_()
    {
        auto bias_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
        bias_pub_ = this->create_publisher<visual_inertial::msg::ImuBias>("imu_bias", bias_qos);

        if (node_cfg_.publish_optimization_result)
        {
            optimization_result_pub_ = this->create_publisher<visual_inertial::msg::OptimizationResult>(
                node_cfg_.optimization_result_topic, bias_qos);
        }

        if (node_cfg_.localization_mode)
        {
            localization_feedback_pub_ =
                this->create_publisher<visual_inertial::msg::LocalizationFeedback>(
                    node_cfg_.localization_feedback_topic, bias_qos);
        }

        if (node_cfg_.publish_optimized_landmarks)
        {
            auto lm_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
            lm_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(node_cfg_.landmark_topic, lm_qos);
        }
    }

    void initSubscriptions_()
    {
        auto kf_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        auto info_qos = rclcpp::SensorDataQoS();

        left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            node_cfg_.left_info_topic, info_qos,
            std::bind(&OptimizationNode::onLeftCameraInfo_, this, std::placeholders::_1));

        right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            node_cfg_.right_info_topic, info_qos,
            std::bind(&OptimizationNode::onRightCameraInfo_, this, std::placeholders::_1));

        if (node_cfg_.localization_mode)
        {
            localization_cmd_sub_ =
                create_subscription<visual_inertial::msg::LocalizationCommand>(
                    node_cfg_.localization_command_topic,
                    kf_qos,
                    std::bind(&OptimizationNode::onLocalizationCommand_, this, std::placeholders::_1));
        }

        kf_sub_ = create_subscription<visual_inertial::msg::Keyframe>(
            node_cfg_.keyframe_topic, kf_qos,
            std::bind(&OptimizationNode::onKeyframe_, this, std::placeholders::_1));
    }

    void startWorker_()
    {
        stop_.store(false);
        worker_ = std::thread([this]()
                              { workerLoop_(); });
    }

    void logStartup_() const
    {
        RCLCPP_INFO(get_logger(),
                    "Optimization node started. mode=%s KF=%s L_info=%s R_info=%s TF=%s->%s @ %.1f Hz",
                    node_cfg_.operation_mode.c_str(),
                    node_cfg_.keyframe_topic.c_str(), node_cfg_.left_info_topic.c_str(), node_cfg_.right_info_topic.c_str(),
                    node_cfg_.map_frame_id.c_str(), node_cfg_.odom_frame_id.c_str(), node_cfg_.tf_pub_rate_hz);

        if (node_cfg_.publish_optimized_landmarks)
        {
            RCLCPP_INFO(get_logger(), "Optimized landmark publishing enabled on '%s'",
                        node_cfg_.landmark_topic.c_str());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Optimized landmark publishing disabled");
        }

        if (node_cfg_.publish_optimization_result)
        {
            RCLCPP_INFO(get_logger(), "Optimization result publishing enabled on '%s'",
                        node_cfg_.optimization_result_topic.c_str());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Optimization result publishing disabled");
        }

        if (node_cfg_.localization_mode)
        {
            RCLCPP_INFO(
                get_logger(),
                "Localization command topic='%s' feedback topic='%s' wait=%.1f ms",
                node_cfg_.localization_command_topic.c_str(),
                node_cfg_.localization_feedback_topic.c_str(),
                node_cfg_.localization_command_wait_ms);
        }
    }

    // Backend bring-up
    bool haveStereoCalibration_() const
    {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        return left_info_.has_value() && right_info_.has_value();
    }

    bool ensureBodyCameraExtrinsicReady_()
    {
        if (!node_cfg_.auto_resolve_t_bc_from_tf || t_bc_resolved_from_tf_)
        {
            return true;
        }

        std::string camera_frame;
        {
            std::lock_guard<std::mutex> lk(calib_mtx_);
            if (!left_info_.has_value())
            {
                return false;
            }
            camera_frame = left_info_->header.frame_id;
        }

        if (camera_frame.empty())
        {
            return false;
        }

        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                node_cfg_.body_frame_id, camera_frame, tf2::TimePointZero);
            optimization_->setBodyCameraExtrinsic(
                visual_inertial::transport::transformMsgToIso(tf.transform));
            t_bc_resolved_from_tf_ = true;

            const auto &T_BC = optimization_->config().T_BC;
            Eigen::Quaterniond q(T_BC.rotation());
            q.normalize();
            RCLCPP_INFO(
                get_logger(),
                "Resolved T_BC from TF: %s <- %s, t=[%.6f %.6f %.6f], q_xyzw=[%.6f %.6f %.6f %.6f]",
                node_cfg_.body_frame_id.c_str(), camera_frame.c_str(),
                T_BC.translation().x(), T_BC.translation().y(), T_BC.translation().z(),
                q.x(), q.y(), q.z(), q.w());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for TF %s <- %s to resolve T_BC: %s",
                node_cfg_.body_frame_id.c_str(), camera_frame.c_str(), ex.what());
            return false;
        }
    }

    std::optional<CameraRig> makeCameraRig_() const
    {
        sensor_msgs::msg::CameraInfo left_info;
        sensor_msgs::msg::CameraInfo right_info;
        {
            std::lock_guard<std::mutex> lk(calib_mtx_);
            if (!left_info_.has_value() || !right_info_.has_value())
            {
                return std::nullopt;
            }
            left_info = *left_info_;
            right_info = *right_info_;
        }

        auto rig = makeStereoModel(left_info, right_info);
        if (!rig.valid())
        {
            return std::nullopt;
        }

        return rig;
    }

    void unsubscribeCameraInfo_()
    {
        left_info_sub_.reset();
        right_info_sub_.reset();
    }

    void maybeInitRigAndOptimizer_()
    {
        if (optimization_->ready())
        {
            return;
        }
        if (!haveStereoCalibration_())
        {
            return;
        }
        if (!ensureBodyCameraExtrinsicReady_())
        {
            return;
        }

        const auto rig = makeCameraRig_();
        if (!rig.has_value())
        {
            RCLCPP_WARN(get_logger(), "Stereo rig computed but invalid (baseline/intrinsics). Waiting...");
            return;
        }

        if (!optimization_->initializeRig(*rig))
        {
            return;
        }

        unsubscribeCameraInfo_();

        RCLCPP_INFO(get_logger(),
                    "Stereo rig ready: fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.5f (m). Optimizer initialized.",
                    rig->left.fx(), rig->left.fy(), rig->left.cx(), rig->left.cy(), rig->baseline);
    }

    // ROS callbacks
    void onLeftCameraInfo_(sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lk(calib_mtx_);
            left_info_ = *msg;
        }
        maybeInitRigAndOptimizer_();
    }

    void onRightCameraInfo_(sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lk(calib_mtx_);
            right_info_ = *msg;
        }
        maybeInitRigAndOptimizer_();
    }

    void onKeyframe_(visual_inertial::msg::Keyframe::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lk(q_mtx_);
            kf_q_.push_back(*msg);
            while (kf_q_.size() > node_cfg_.max_keyframe_queue)
                kf_q_.pop_front();
        }
        q_cv_.notify_one();
    }

    void onLocalizationCommand_(visual_inertial::msg::LocalizationCommand::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lk(localization_cmd_mtx_);
            localization_cmd_by_kf_[msg->kf_id] = *msg;
        }
        localization_cmd_cv_.notify_all();
    }

    // Worker/runtime flow
    void workerLoop_()
    {
        while (rclcpp::ok() && !stop_.load())
        {
            visual_inertial::msg::Keyframe msg;

            {
                std::unique_lock<std::mutex> lk(q_mtx_);
                q_cv_.wait(lk, [&]
                           { return stop_.load() || !kf_q_.empty(); });
                if (stop_.load())
                    break;

                msg = std::move(kf_q_.front());
                kf_q_.pop_front();

                RCLCPP_INFO(this->get_logger(),
                            "Consumed keyframe kf_id=%lu, remaining queue size=%zu",
                            static_cast<unsigned long>(msg.kf_id),
                            kf_q_.size());
            }

            if (!optimization_->ready())
            {
                static uint64_t warn_ctr = 0;
                if ((warn_ctr++ % 50) == 0)
                {
                    RCLCPP_WARN(get_logger(), "Waiting for CameraInfo calibration; dropping keyframes for now...");
                }
                continue;
            }

            if (hasMalformedTrackPayload_(msg))
            {
                logMalformedTrackPayload_(msg);
            }

            const auto update = prepareOptimizationUpdate_(msg);
            applyLocalizationDecision_(update, *optimization_);

            const auto optimize_start = std::chrono::steady_clock::now();
            const auto res = optimization_->push(
                update.event,
                update.between_meas,
                update.absolute_pose_priors,
                update.T_WB_init_override,
                update.T_WB_anchor_override);
            const auto optimize_end = std::chrono::steady_clock::now();
            const double optimize_ms =
                std::chrono::duration<double, std::milli>(optimize_end - optimize_start).count();
            if (!res)
                continue;

            handleOptimizationResult_(msg, *res, *optimization_, optimize_ms);
        }
    }

    // Output publishing and map/odom state updates
    void publishMapOdomTf_()
    {
        Eigen::Isometry3d T_map_odom;
        const rclcpp::Time now = this->now();

        {
            std::lock_guard<std::mutex> lk(tf_mtx_);
            if (!have_target_)
                return;
            T_map_odom = T_map_odom_target_;
        }

        // publish TF outside the lock
        auto tf = visual_inertial::transport::isoToTf(
            T_map_odom, now, node_cfg_.map_frame_id, node_cfg_.odom_frame_id);
        tf_broadcaster_->sendTransform(tf);
    }

    void publishLandmarks_(OptimizationModule &optimization)
    {
        if (!lm_pub_)
            return;

        const auto lms = optimization.getLandmarks(node_cfg_.lm_fetch_max);
        lm_pub_->publish(
            visual_inertial::transport::makeLandmarkPointCloudMsg(
                lms, this->now(), node_cfg_.map_frame_id));
    }

private:
    // Configuration and library-facing runtime components
    std::unique_ptr<visual_inertial::OptimizationNodeParamHandler> param_handler_;
    visual_inertial::OptimizationNodeConfig node_cfg_;
    std::unique_ptr<OptimizationModule> optimization_;

    // TF interfaces and latest published map->odom state
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool t_bc_resolved_from_tf_{false};
    std::mutex tf_mtx_;
    Eigen::Isometry3d T_map_odom_target_ = Eigen::Isometry3d::Identity();
    bool have_target_ = false;

    // Calibration inputs used to bring the backend online
    mutable std::mutex calib_mtx_;
    std::optional<sensor_msgs::msg::CameraInfo> left_info_;
    std::optional<sensor_msgs::msg::CameraInfo> right_info_;

    // ROS interfaces
    rclcpp::Subscription<visual_inertial::msg::Keyframe>::SharedPtr kf_sub_;
    rclcpp::Subscription<visual_inertial::msg::LocalizationCommand>::SharedPtr localization_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
    rclcpp::Publisher<visual_inertial::msg::ImuBias>::SharedPtr bias_pub_;
    rclcpp::Publisher<visual_inertial::msg::LocalizationFeedback>::SharedPtr localization_feedback_pub_;
    rclcpp::Publisher<visual_inertial::msg::OptimizationResult>::SharedPtr optimization_result_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lm_pub_;

    // Localization command ingress
    std::mutex localization_cmd_mtx_;
    std::condition_variable localization_cmd_cv_;
    std::unordered_map<uint64_t, visual_inertial::msg::LocalizationCommand> localization_cmd_by_kf_;

    // Worker-side keyframe queue and thread
    std::mutex q_mtx_;
    std::condition_variable q_cv_;
    std::deque<visual_inertial::msg::Keyframe> kf_q_;
    std::thread worker_;
    std::atomic<bool> stop_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptimizationNode>());
    rclcpp::shutdown();
    return 0;
}
