#include <rclcpp/rclcpp.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/optimization_result.hpp>
#include <visual_inertial/optimization_node_params.hpp>

#include <visual_inertial_common/types.hpp>
#include <visual_inertial_localization/localization.hpp>
#include <visual_inertial_optimization/optimization.hpp>
#include <visual_inertial_optimization/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
#include <vector>

#include <visual_inertial/msg/imu_bias.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace
{

    static Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose &p)
    {
        Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        q.normalize();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        return T;
    }

    static geometry_msgs::msg::TransformStamped isoToTf(
        const Eigen::Isometry3d &T_parent_child,
        const rclcpp::Time &stamp,
        const std::string &parent_frame,
        const std::string &child_frame)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = parent_frame;
        tf.child_frame_id = child_frame;

        tf.transform.translation.x = T_parent_child.translation().x();
        tf.transform.translation.y = T_parent_child.translation().y();
        tf.transform.translation.z = T_parent_child.translation().z();

        Eigen::Quaterniond q(T_parent_child.linear());
        q.normalize();
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        return tf;
    }

    static geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d &T)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = T.translation().x();
        pose.position.y = T.translation().y();
        pose.position.z = T.translation().z();

        Eigen::Quaterniond q(T.linear());
        q.normalize();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();

        return pose;
    }

    static Eigen::Isometry3d transformMsgToIso(const geometry_msgs::msg::Transform &t)
    {
        Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
        q.normalize();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(
            t.translation.x,
            t.translation.y,
            t.translation.z);
        return T;
    }

    static AbsolutePosePrior toAbsolutePosePrior(
        const visual_inertial_localization::LocalizationPosePrior &prior)
    {
        AbsolutePosePrior out;
        out.T_WB = prior.T_WB;
        out.rot_sigma_rad = prior.rot_sigma_rad;
        out.trans_sigma_m = prior.trans_sigma_m;
        out.huber_k = prior.huber_k;
        return out;
    }

    static KeyframeEvent toKeyframeEvent(const visual_inertial::msg::Keyframe &msg)
    {
        KeyframeEvent ev;
        ev.kf_id = msg.kf_id;
        ev.t_start = msg.t_start; // rclcpp::Time(msg.header.stamp).seconds();
        ev.t_end = msg.t_end;

        // Frontend-exported pose: odom/startup frame <- body
        ev.T_OB = poseMsgToIso(msg.pose_odom_body);
        ev.has_vo_between = (msg.has_vo_between != 0);
        if (ev.has_vo_between)
        {
            ev.T_Bkm1_Bk = poseMsgToIso(msg.between_pose_prev_curr);
        }
        ev.interval_health.num_frames = msg.interval_health.num_frames;
        ev.interval_health.num_pose_valid_frames = msg.interval_health.num_pose_valid_frames;
        ev.interval_health.num_degraded_frames = msg.interval_health.num_degraded_frames;
        ev.interval_health.num_lost_frames = msg.interval_health.num_lost_frames;
        ev.interval_health.min_tracks = msg.interval_health.min_tracks;
        ev.interval_health.mean_tracks = msg.interval_health.mean_tracks;
        ev.interval_health.min_track_retention = msg.interval_health.min_track_retention;
        ev.interval_health.mean_track_retention = msg.interval_health.mean_track_retention;
        ev.interval_health.mean_pnp_inlier_ratio = msg.interval_health.mean_pnp_inlier_ratio;
        ev.interval_health.max_pnp_reproj_rmse_px = msg.interval_health.max_pnp_reproj_rmse_px;
        ev.interval_health.min_track_coverage = msg.interval_health.min_track_coverage;
        ev.interval_health.mean_track_coverage = msg.interval_health.mean_track_coverage;

        const size_t n = msg.track_ids.size();
        if (msg.u_l.size() != n || msg.v_l.size() != n ||
            msg.u_r.size() != n || msg.v_r.size() != n ||
            msg.has_right.size() != n)
        {
            ev.ids.clear();
            ev.pl.clear();
            ev.pr.clear();
            ev.has_r.clear();
            return ev;
        }

        ev.ids.resize(n);
        ev.pl.resize(n);
        ev.pr.resize(n);
        ev.has_r.resize(n);

        for (size_t i = 0; i < n; ++i)
        {
            ev.ids[i] = msg.track_ids[i];
            ev.pl[i] = cv::Point2f(msg.u_l[i], msg.v_l[i]);
            ev.pr[i] = cv::Point2f(msg.u_r[i], msg.v_r[i]);
            ev.has_r[i] = msg.has_right[i];
        }

        ev.has_imu = (msg.has_imu != 0);
        ev.pim_bytes = msg.pim_bytes; // uint8[] -> std::vector<uint8_t>

        return ev;
    }

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
        opt_cfg_ = params.optimizer;

        if (node_cfg_.localization_mode)
        {
            if (!params.localization.has_value())
            {
                throw std::runtime_error(
                    "optimization node missing localization config in localization mode");
            }

            localization_ = std::make_unique<visual_inertial_localization::LocalizationModule>(
                *params.localization);
            const auto report = localization_->loadTagMap();
            if (localization_->config().tag_map_path.empty())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "No tag priors were provided; running in pure odometry mode");
            }
            else if (report.ok)
            {
                localization_use_tag_priors_ = true;
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
                RCLCPP_INFO(
                    get_logger(),
                    "No tag priors were provided; running in pure odometry mode");
            }
        }

        // -------- TF broadcaster --------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // -------- TF timer (constant-rate rebroadcast of latest T_map_odom) --------
        using namespace std::chrono_literals;
        tf_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / node_cfg_.tf_pub_rate_hz),
            [this]()
            { this->publishFilteredTf_(); });

        // -------- QoS --------
        auto kf_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        auto info_qos = rclcpp::SensorDataQoS();
        auto tag_qos = rclcpp::SensorDataQoS();

        // -------- Subscriptions: CameraInfo --------
        left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            node_cfg_.left_info_topic, info_qos,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(calib_mtx_);
                left_info_ = *msg;
                maybeInitRigAndOptimizer_();
            });

        right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            node_cfg_.right_info_topic, info_qos,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(calib_mtx_);
                right_info_ = *msg;
                maybeInitRigAndOptimizer_();
            });

        if (node_cfg_.localization_mode && localization_)
        {
            tag_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                localization_->config().tag_topic,
                tag_qos,
                [this](apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
                {
                    this->onTagDetections_(std::move(msg));
                });
        }

        // -------- Subscription: Keyframes --------
        kf_sub_ = create_subscription<visual_inertial::msg::Keyframe>(
            node_cfg_.keyframe_topic, kf_qos,
            [this](visual_inertial::msg::Keyframe::SharedPtr msg)
            {
                {
                    std::lock_guard<std::mutex> lk(q_mtx_);
                    kf_q_.push_back(*msg);
                    while (kf_q_.size() > node_cfg_.max_keyframe_queue)
                        kf_q_.pop_front();
                }
                q_cv_.notify_one();
            });

        // -------- Worker thread --------
        stop_.store(false);
        worker_ = std::thread([this]()
                              { workerLoop_(); });

        RCLCPP_INFO(get_logger(),
                    "Optimization node started. mode=%s KF=%s L_info=%s R_info=%s TF=%s->%s @ %.1f Hz",
                    node_cfg_.operation_mode.c_str(),
                    node_cfg_.keyframe_topic.c_str(), node_cfg_.left_info_topic.c_str(), node_cfg_.right_info_topic.c_str(),
                    node_cfg_.map_frame_id.c_str(), node_cfg_.odom_frame_id.c_str(), node_cfg_.tf_pub_rate_hz);

        // publisher for imu bias
        auto bias_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
        bias_pub_ = this->create_publisher<visual_inertial::msg::ImuBias>("imu_bias", bias_qos);
        if (node_cfg_.publish_optimization_result)
        {
            optimization_result_pub_ = this->create_publisher<visual_inertial::msg::OptimizationResult>(
                node_cfg_.optimization_result_topic, bias_qos);
        }

        // Landmark publishing
        if (node_cfg_.publish_optimized_landmarks)
        {
            auto lm_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
            lm_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(node_cfg_.landmark_topic, lm_qos);
        }

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
        std::optional<visual_inertial_localization::LocalizationDecision> localization_decision;
    };

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

    PreparedOptimizationUpdate prepareOptimizationUpdate_(
        const visual_inertial::msg::Keyframe &msg) const
    {
        PreparedOptimizationUpdate update;
        update.event = toKeyframeEvent(msg);

        if (update.event.has_vo_between)
        {
            update.between_meas = update.event.T_Bkm1_Bk;
        }

        if (!node_cfg_.localization_mode ||
            !localization_ ||
            !localization_use_tag_priors_)
        {
            return update;
        }

        update.localization_decision = localization_->processKeyframe(
            rclcpp::Time(msg.header.stamp).nanoseconds(),
            update.event.T_OB);

        for (const auto &prior : update.localization_decision->optimizer_inputs.absolute_pose_priors)
        {
            update.absolute_pose_priors.push_back(toAbsolutePosePrior(prior));
        }
        update.T_WB_init_override =
            update.localization_decision->optimizer_inputs.T_WB_init_override;
        update.T_WB_anchor_override =
            update.localization_decision->optimizer_inputs.T_WB_anchor_override;

        return update;
    }

    void applyLocalizationDecision_(
        const PreparedOptimizationUpdate &update,
        const std::shared_ptr<Optimizer> &opt)
    {
        if (!update.localization_decision.has_value())
        {
            return;
        }

        const auto &decision = *update.localization_decision;

        if (decision.action == visual_inertial_localization::LocalizationAction::Bootstrap)
        {
            opt->reset();
            if (decision.bootstrap.has_value())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Localization bootstrap established: support=%zu score=%.1f; switching from odometry-only to map-localized tracking",
                    decision.bootstrap->support_count,
                    decision.bootstrap->score);
            }
        }
        else if (decision.waiting_for_bootstrap)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Localization mode has not seen a stable mapped tag yet; running in pure odometry until first bootstrap");
        }

        if (decision.action == visual_inertial_localization::LocalizationAction::Relocalize)
        {
            opt->reset();
            RCLCPP_INFO(
                get_logger(),
                "Localization graph reset for relocalization at incoming kf_id=%lu",
                static_cast<unsigned long>(update.event.kf_id));

            if (decision.relocalization.has_value())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Localization relocalization trigger: history_frames=%zu support=%zu map_odom_trans_error=%.3f m map_odom_rot_error=%.1f deg",
                    decision.relocalization->frame_support,
                    decision.relocalization->support_count,
                    decision.relocalization->translation_error_m,
                    decision.relocalization->rotation_error_rad * 180.0 / M_PI);
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
        visual_inertial::msg::ImuBias bmsg;
        bmsg.header.stamp = msg.header.stamp;
        bmsg.header.frame_id = "base_link";
        bmsg.kf_id = result.kf_id;

        bmsg.accel_bias.x = result.bias_opt.accel.x();
        bmsg.accel_bias.y = result.bias_opt.accel.y();
        bmsg.accel_bias.z = result.bias_opt.accel.z();

        bmsg.gyro_bias.x = result.bias_opt.gyro.x();
        bmsg.gyro_bias.y = result.bias_opt.gyro.y();
        bmsg.gyro_bias.z = result.bias_opt.gyro.z();

        bias_pub_->publish(bmsg);
    }

    void publishOptimizationResult_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result)
    {
        if (!optimization_result_pub_)
        {
            return;
        }

        visual_inertial::msg::OptimizationResult opt_msg;
        opt_msg.header.stamp = msg.header.stamp;
        opt_msg.header.frame_id = node_cfg_.map_frame_id;
        opt_msg.kf_id = result.kf_id;
        opt_msg.t_s = result.t_s;
        opt_msg.pose_wc_opt = isoToPoseMsg(result.T_WC_opt);
        opt_msg.pose_wb_opt = isoToPoseMsg(result.T_WB_opt);
        opt_msg.active_keyframe_poses.reserve(result.active_keyframe_poses.size());
        for (const auto &active_pose : result.active_keyframe_poses)
        {
            visual_inertial::msg::OptimizedKeyframePose active_pose_msg;
            active_pose_msg.kf_id = active_pose.kf_id;
            active_pose_msg.pose_wc_opt = isoToPoseMsg(active_pose.T_WC_opt);
            active_pose_msg.pose_wb_opt = isoToPoseMsg(active_pose.T_WB_opt);
            opt_msg.active_keyframe_poses.push_back(std::move(active_pose_msg));
        }
        opt_msg.stats.num_keyframes_in_window = result.stats.num_keyframes_in_window;
        opt_msg.stats.num_landmarks_alive = result.stats.num_landmarks_alive;
        opt_msg.stats.num_landmarks_created = result.stats.num_landmarks_created;
        opt_msg.stats.num_stereo_factors_added = result.stats.num_stereo_factors_added;
        opt_msg.stats.num_imu_factors_added = result.stats.num_imu_factors_added;
        opt_msg.stats.num_between_factors_added = result.stats.num_between_factors_added;
        opt_msg.stats.num_prior_factors_added = result.stats.num_prior_factors_added;
        opt_msg.stats.had_vo_between_measurement = result.stats.had_vo_between_measurement;
        opt_msg.stats.used_vo_between_factor = result.stats.used_vo_between_factor;
        opt_msg.stats.skipped_vo_between_factor = result.stats.skipped_vo_between_factor;
        opt_msg.stats.vo_between_quality = result.stats.vo_between_quality;
        opt_msg.stats.vo_between_sigma_scale = result.stats.vo_between_sigma_scale;
        opt_msg.stats.imu_only_update = result.stats.imu_only_update;
        opt_msg.stats.update_iterations = result.stats.update_iterations;
        opt_msg.stats.update_intermediate_steps = result.stats.update_intermediate_steps;
        opt_msg.stats.update_nonlinear_variables = result.stats.update_nonlinear_variables;
        opt_msg.stats.update_linear_variables = result.stats.update_linear_variables;
        opt_msg.stats.final_error = result.stats.final_error;
        opt_msg.stats.has_error_before = result.stats.has_error_before;
        opt_msg.stats.error_before = result.stats.error_before;
        opt_msg.stats.has_error_after = result.stats.has_error_after;
        opt_msg.stats.error_after = result.stats.error_after;
        opt_msg.stats.variables_relinearized = result.stats.variables_relinearized;
        opt_msg.stats.variables_reeliminated = result.stats.variables_reeliminated;
        opt_msg.stats.factors_recalculated = result.stats.factors_recalculated;
        opt_msg.stats.cliques = result.stats.cliques;
        opt_msg.has_velocity = result.has_velocity;
        opt_msg.velocity_opt.x = result.velocity_opt.x();
        opt_msg.velocity_opt.y = result.velocity_opt.y();
        opt_msg.velocity_opt.z = result.velocity_opt.z();
        opt_msg.accel_bias.x = result.bias_opt.accel.x();
        opt_msg.accel_bias.y = result.bias_opt.accel.y();
        opt_msg.accel_bias.z = result.bias_opt.accel.z();
        opt_msg.gyro_bias.x = result.bias_opt.gyro.x();
        opt_msg.gyro_bias.y = result.bias_opt.gyro.y();
        opt_msg.gyro_bias.z = result.bias_opt.gyro.z();
        opt_msg.has_pose_wb_covariance = result.has_pose_wb_covariance;
        opt_msg.pose_wb_covariance = result.pose_wb_covariance;
        opt_msg.has_velocity_covariance = result.has_velocity_covariance;
        opt_msg.velocity_covariance = result.velocity_covariance;
        opt_msg.has_bias_covariance = result.has_bias_covariance;
        opt_msg.bias_covariance = result.bias_covariance;
        optimization_result_pub_->publish(opt_msg);
    }

    Eigen::Isometry3d updateMapOdomTarget_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result)
    {
        const Eigen::Isometry3d T_odom_B = poseMsgToIso(msg.pose_odom_body);
        const Eigen::Isometry3d T_map_B = result.T_WB_opt;
        const Eigen::Isometry3d T_map_odom = T_map_B * T_odom_B.inverse();

        {
            std::lock_guard<std::mutex> lk(tf_mtx_);
            T_map_odom_target_ = T_map_odom;
            have_target_ = true;

            if (!have_filt_)
            {
                T_map_odom_filt_ = T_map_odom_target_;
                have_filt_ = true;
                last_filt_update_ = this->now();
            }
        }

        return T_map_odom;
    }

    void updateLocalizationMapOdom_(const Eigen::Isometry3d &T_map_odom)
    {
        if (localization_ &&
            localization_->state() ==
                visual_inertial_localization::LocalizationState::Localized)
        {
            localization_->updateMapOdomEstimate(T_map_odom);
        }
    }

    void handleOptimizationResult_(
        const visual_inertial::msg::Keyframe &msg,
        const OptimizationResult &result,
        const std::shared_ptr<Optimizer> &opt,
        double optimize_ms)
    {
        logOptimizationUpdate_(result, optimize_ms);
        publishImuBias_(msg, result);
        publishOptimizationResult_(msg, result);
        publishLandmarks_(opt);
        const auto T_map_odom = updateMapOdomTarget_(msg, result);
        updateLocalizationMapOdom_(T_map_odom);
        have_map_odom_.store(true);
    }
    void maybeInitRigAndOptimizer_()
    {
        if (rig_ready_)
            return;
        if (!left_info_.has_value() || !right_info_.has_value())
            return;

        if (!maybeResolveBodyCameraExtrinsic_())
            return;

        const auto rig = makeStereoModel(*left_info_, *right_info_);
        if (!rig.valid())
        {
            RCLCPP_WARN(get_logger(), "Stereo rig computed but invalid (baseline/intrinsics). Waiting...");
            return;
        }

        opt_cfg_.rig = rig;

        {
            std::lock_guard<std::mutex> lk(opt_mtx_);
            optimizer_ = std::make_shared<Optimizer>(opt_cfg_);
        }
        rig_ready_ = true;

        RCLCPP_INFO(get_logger(),
                    "Stereo rig ready: fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.5f (m). Optimizer initialized.",
                    rig.left.fx(), rig.left.fy(), rig.left.cx(), rig.left.cy(), rig.baseline);
    }

    bool maybeResolveBodyCameraExtrinsic_()
    {
        if (!node_cfg_.auto_resolve_t_bc_from_tf || t_bc_resolved_from_tf_)
        {
            return true;
        }
        if (!left_info_.has_value())
        {
            return false;
        }

        const std::string &camera_frame = left_info_->header.frame_id;
        if (camera_frame.empty())
        {
            return false;
        }

        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                node_cfg_.body_frame_id, camera_frame, tf2::TimePointZero);
            opt_cfg_.T_BC = transformMsgToIso(tf.transform);
            t_bc_resolved_from_tf_ = true;

            Eigen::Quaterniond q(opt_cfg_.T_BC.rotation());
            q.normalize();
            RCLCPP_INFO(
                get_logger(),
                "Resolved T_BC from TF: %s <- %s, t=[%.6f %.6f %.6f], q_xyzw=[%.6f %.6f %.6f %.6f]",
                node_cfg_.body_frame_id.c_str(), camera_frame.c_str(),
                opt_cfg_.T_BC.translation().x(), opt_cfg_.T_BC.translation().y(), opt_cfg_.T_BC.translation().z(),
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

            if (!rig_ready_)
            {
                static uint64_t warn_ctr = 0;
                if ((warn_ctr++ % 50) == 0)
                {
                    RCLCPP_WARN(get_logger(), "Waiting for CameraInfo calibration; dropping keyframes for now...");
                }
                continue;
            }

            std::shared_ptr<Optimizer> opt;
            {
                std::lock_guard<std::mutex> lk(opt_mtx_);
                opt = optimizer_;
            }
            if (!opt)
                continue;

            if (hasMalformedTrackPayload_(msg))
            {
                logMalformedTrackPayload_(msg);
            }

            const auto update = prepareOptimizationUpdate_(msg);
            applyLocalizationDecision_(update, opt);

            const auto optimize_start = std::chrono::steady_clock::now();
            const auto res = opt->push(
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

            handleOptimizationResult_(msg, *res, opt, optimize_ms);
        }
    }

    void publishFilteredTf_()
    {
        Eigen::Isometry3d T_target;
        Eigen::Isometry3d T_filt;
        rclcpp::Time now = this->now();

        {
            std::lock_guard<std::mutex> lk(tf_mtx_);
            if (!have_target_ || !have_filt_)
                return;

            // compute alpha from dt and time constant
            double dt = (now - last_filt_update_).seconds();
            if (dt <= 0.0)
                dt = 1.0 / node_cfg_.publish_tf_hz;
            last_filt_update_ = now;

            double alpha = dt / (node_cfg_.smooth_tau_s + dt); // EMA equivalent
            if (alpha < 0.0)
                alpha = 0.0;
            if (alpha > 1.0)
                alpha = 1.0;

            // --- translation EMA ---
            T_map_odom_filt_.translation() =
                (1.0 - alpha) * T_map_odom_filt_.translation() +
                alpha * T_map_odom_target_.translation();

            // --- rotation slerp ---
            Eigen::Quaterniond q_old(T_map_odom_filt_.linear());
            Eigen::Quaterniond q_new(T_map_odom_target_.linear());
            q_old.normalize();
            q_new.normalize();
            Eigen::Quaterniond q_f = q_old.slerp(alpha, q_new);
            q_f.normalize();
            T_map_odom_filt_.linear() = q_f.toRotationMatrix();

            T_filt = T_map_odom_filt_;
        }

        // publish TF outside the lock
        auto tf = isoToTf(T_filt, now, node_cfg_.map_frame_id, node_cfg_.odom_frame_id);
        tf_broadcaster_->sendTransform(tf);
    }

    void publishLandmarks_(const std::shared_ptr<Optimizer> &opt)
    {
        if (!lm_pub_)
            return;

        const auto lms = opt->getLandmarks(node_cfg_.lm_fetch_max);

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = node_cfg_.map_frame_id;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(lms.size());
        msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(lms.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

        for (const auto &lm : lms)
        {
            *it_x = static_cast<float>(lm.p_W.x());
            *it_y = static_cast<float>(lm.p_W.y());
            *it_z = static_cast<float>(lm.p_W.z());
            ++it_x;
            ++it_y;
            ++it_z;
        }

        lm_pub_->publish(msg);
    }

    void onTagDetections_(apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if (!node_cfg_.localization_mode || !localization_ || !localization_use_tag_priors_ || !tf_buffer_ || !msg)
        {
            return;
        }

        const auto report = localization_->ingestDetections(
            *msg, node_cfg_.body_frame_id, node_cfg_.odom_frame_id, *tf_buffer_);
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
    std::unique_ptr<visual_inertial::OptimizationNodeParamHandler> param_handler_;
    visual_inertial::OptimizationNodeConfig node_cfg_;
    bool localization_use_tag_priors_{false};
    std::unique_ptr<visual_inertial_localization::LocalizationModule> localization_;

    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Subscriptions
    rclcpp::Subscription<visual_inertial::msg::Keyframe>::SharedPtr kf_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool t_bc_resolved_from_tf_{false};

    // Latest map->odom cached transform
    std::mutex tf_mtx_;
    std::atomic<bool> have_map_odom_{false};

    // Calibration state
    std::mutex calib_mtx_;
    std::optional<sensor_msgs::msg::CameraInfo> left_info_;
    std::optional<sensor_msgs::msg::CameraInfo> right_info_;
    std::atomic<bool> rig_ready_{false};

    // Backend
    OptimizationConfig opt_cfg_;
    std::mutex opt_mtx_;
    std::shared_ptr<Optimizer> optimizer_;

    // Queue / threading
    std::mutex q_mtx_;
    std::condition_variable q_cv_;
    std::deque<visual_inertial::msg::Keyframe> kf_q_;

    std::thread worker_;
    std::atomic<bool> stop_{false};

    // publisher for imu bias
    rclcpp::Publisher<visual_inertial::msg::ImuBias>::SharedPtr bias_pub_;
    rclcpp::Publisher<visual_inertial::msg::OptimizationResult>::SharedPtr optimization_result_pub_;

    // smoother for t map odom
    Eigen::Isometry3d T_map_odom_target_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_map_odom_filt_ = Eigen::Isometry3d::Identity();
    bool have_target_ = false;
    bool have_filt_ = false;

    rclcpp::Time last_filt_update_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lm_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptimizationNode>());
    rclcpp::shutdown();
    return 0;
}
