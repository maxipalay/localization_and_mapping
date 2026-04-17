#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/optimization_result.hpp>

#include <visual_inertial_common/types.hpp>
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
#include <algorithm>
#include <chrono>
#include <unordered_map>
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
        // -------- Parameters --------
        keyframe_topic_ = declare_parameter<std::string>("keyframe_topic", "keyframes");
        left_info_topic_ = declare_parameter<std::string>("left_info_topic", "oak/left/camera_info");
        right_info_topic_ = declare_parameter<std::string>("right_info_topic", "oak/right/camera_info");

        map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
        body_frame_id_ = declare_parameter<std::string>("body_frame_id", "body");
        auto_resolve_t_bc_from_tf_ =
            declare_parameter<bool>("auto_resolve_t_bc_from_tf", true);

        // Broadcast rate for map->odom
        tf_pub_rate_hz_ = declare_parameter<double>("tf_pub_rate_hz", 30.0);

        // Backend config params
        cfg_.window_size = static_cast<size_t>(declare_parameter<int>("window_size", 8));
        cfg_.stereo_sigma_px = declare_parameter<double>("stereo_sigma_px", 1.0);
        cfg_.stereo_huber_k = declare_parameter<double>("stereo_huber_k", cfg_.stereo_huber_k);

        cfg_.prior_rot_sigma_rad = declare_parameter<double>("prior_rot_sigma_rad", cfg_.prior_rot_sigma_rad);
        cfg_.prior_trans_sigma_m = declare_parameter<double>("prior_trans_sigma_m", cfg_.prior_trans_sigma_m);

        cfg_.vel_prior_sigma = declare_parameter<double>("vel_prior_sigma", cfg_.vel_prior_sigma);
        cfg_.bias_acc_prior_sigma = declare_parameter<double>("bias_acc_prior_sigma", cfg_.bias_acc_prior_sigma);
        cfg_.bias_gyro_prior_sigma = declare_parameter<double>("bias_gyro_prior_sigma", cfg_.bias_gyro_prior_sigma);

        cfg_.use_vo_between = declare_parameter<bool>("use_vo_between", cfg_.use_vo_between);
        cfg_.between_rot_sigma_rad = declare_parameter<double>("between_rot_sigma_rad", cfg_.between_rot_sigma_rad);
        cfg_.between_trans_sigma_m = declare_parameter<double>("between_trans_sigma_m", cfg_.between_trans_sigma_m);
        cfg_.between_huber_k = declare_parameter<double>("between_huber_k", cfg_.between_huber_k);

        cfg_.use_imu = declare_parameter<bool>("use_imu", cfg_.use_imu);

        cfg_.init_landmarks_from_stereo = declare_parameter<bool>("init_landmarks_from_stereo", true);
        cfg_.prune_unobserved_landmarks = declare_parameter<bool>("prune_unobserved_landmarks", true);

        // Optional override for body<-camera extrinsics
        Eigen::Quaterniond q_default(cfg_.T_BC.rotation());
        q_default.normalize();
        auto t_bc = declare_parameter<std::vector<double>>(
            "T_BC_translation", {cfg_.T_BC.translation().x(),
                                 cfg_.T_BC.translation().y(),
                                 cfg_.T_BC.translation().z()});
        auto q_bc_xyzw = declare_parameter<std::vector<double>>(
            "T_BC_quaternion_xyzw", {q_default.x(), q_default.y(), q_default.z(), q_default.w()});
        if (t_bc.size() == 3 && q_bc_xyzw.size() == 4)
        {
            Eigen::Quaterniond q(q_bc_xyzw[3], q_bc_xyzw[0], q_bc_xyzw[1], q_bc_xyzw[2]);
            if (q.norm() > 1e-9)
            {
                q.normalize();
                Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
                T.linear() = q.toRotationMatrix();
                T.translation() = Eigen::Vector3d(t_bc[0], t_bc[1], t_bc[2]);
                cfg_.T_BC = T;
            }
            else
            {
                RCLCPP_WARN(get_logger(), "T_BC quaternion has near-zero norm; using default extrinsics");
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "T_BC params malformed (need 3 translation + 4 quaternion entries); using defaults");
        }

        max_queue_ = static_cast<size_t>(declare_parameter<int>("max_keyframe_queue", 30));

        // -------- TF broadcaster --------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // -------- TF timer (constant-rate rebroadcast of latest T_map_odom) --------
        using namespace std::chrono_literals;
        const auto period = (tf_pub_rate_hz_ > 1e-6)
                                ? std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / tf_pub_rate_hz_))
                                : 33ms;

        tf_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / tf_pub_rate_hz_),
            [this]()
            { this->publishFilteredTf_(); });

        // -------- QoS --------
        auto kf_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        auto info_qos = rclcpp::SensorDataQoS();

        // -------- Subscriptions: CameraInfo --------
        left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            left_info_topic_, info_qos,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(calib_mtx_);
                left_info_ = *msg;
                maybeInitRigAndOptimizer_();
            });

        right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            right_info_topic_, info_qos,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(calib_mtx_);
                right_info_ = *msg;
                maybeInitRigAndOptimizer_();
            });

        // -------- Subscription: Keyframes --------
        kf_sub_ = create_subscription<visual_inertial::msg::Keyframe>(
            keyframe_topic_, kf_qos,
            [this](visual_inertial::msg::Keyframe::SharedPtr msg)
            {
                {
                    std::lock_guard<std::mutex> lk(q_mtx_);
                    kf_q_.push_back(*msg);
                    while (kf_q_.size() > max_queue_)
                        kf_q_.pop_front();
                }
                q_cv_.notify_one();
            });

        // -------- Worker thread --------
        stop_.store(false);
        worker_ = std::thread([this]()
                              { workerLoop_(); });

        RCLCPP_INFO(get_logger(),
                    "Optimization node started. KF=%s L_info=%s R_info=%s TF=%s->%s @ %.1f Hz",
                    keyframe_topic_.c_str(), left_info_topic_.c_str(), right_info_topic_.c_str(),
                    map_frame_id_.c_str(), odom_frame_id_.c_str(), tf_pub_rate_hz_);

        // publisher for imu bias
        auto bias_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
        bias_pub_ = this->create_publisher<visual_inertial::msg::ImuBias>("imu_bias", bias_qos);
        publish_optimization_result_ = declare_parameter<bool>("publish_optimization_result", publish_optimization_result_);
        optimization_result_topic_ = declare_parameter<std::string>("optimization_result_topic", optimization_result_topic_);
        if (publish_optimization_result_)
        {
            optimization_result_pub_ = this->create_publisher<visual_inertial::msg::OptimizationResult>(
                optimization_result_topic_, bias_qos);
        }

        // Landmark publishing
        publish_optimized_landmarks_ = declare_parameter<bool>("publish_optimized_landmarks", publish_optimized_landmarks_);
        landmark_topic_ = declare_parameter<std::string>("landmark_topic", landmark_topic_);
        lm_pub_hz_ = declare_parameter<double>("landmark_pub_hz", 2.0);

        if (publish_optimized_landmarks_)
        {
            auto lm_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
            lm_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(landmark_topic_, lm_qos);

            // Timer (publish at constant rate, independent of keyframes)
            lm_timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / std::max(1e-6, lm_pub_hz_)),
                [this]()
                { this->publishLandmarksPointCloud_(); });
        }

        // Optional smoothing params for TF and landmark caching
        smooth_tau_s_ = declare_parameter<double>("smooth_tau_s", smooth_tau_s_);
        publish_tf_hz_ = declare_parameter<double>("publish_tf_hz", publish_tf_hz_);
        lm_cache_max_ = static_cast<size_t>(declare_parameter<int>("lm_cache_max", static_cast<int>(lm_cache_max_)));
        lm_fetch_max_ = static_cast<size_t>(declare_parameter<int>("lm_fetch_max", static_cast<int>(lm_fetch_max_)));

        if (publish_optimized_landmarks_)
        {
            RCLCPP_INFO(get_logger(), "Optimized landmark publishing enabled on '%s' @ %.2f Hz",
                        landmark_topic_.c_str(), lm_pub_hz_);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Optimized landmark publishing disabled");
        }

        if (publish_optimization_result_)
        {
            RCLCPP_INFO(get_logger(), "Optimization result publishing enabled on '%s'",
                        optimization_result_topic_.c_str());
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
    void mergeLandmarksIntoCache_(const std::vector<LandmarkEstimate> &lms)
    {
        std::lock_guard<std::mutex> lk(lm_mtx_);

        for (const auto &e : lms)
        {
            const uint32_t tid = static_cast<uint32_t>(e.tid);
            auto &slot = lm_cache_[tid];
            slot.p_map = e.p_W;
            slot.last_seen_kf = e.last_seen_kf;
        }

        pruneLandmarkCacheLocked_();
    }

    void pruneLandmarkCacheLocked_()
    {
        if (lm_cache_.size() <= lm_cache_max_)
            return;
        if (lm_cache_max_ == 0)
        {
            lm_cache_.clear();
            return;
        }

        // Build list of (tid, last_seen)
        std::vector<std::pair<uint32_t, uint64_t>> items;
        items.reserve(lm_cache_.size());
        for (const auto &kv : lm_cache_)
        {
            items.emplace_back(kv.first, kv.second.last_seen_kf);
        }

        // Find cutoff for newest lm_cache_max_ by last_seen
        auto nth = items.end() - static_cast<std::ptrdiff_t>(lm_cache_max_);
        std::nth_element(items.begin(), nth, items.end(),
                         [](const auto &a, const auto &b)
                         { return a.second < b.second; });
        const uint64_t cutoff = nth->second;

        // Erase everything older than cutoff
        for (auto it = lm_cache_.begin(); it != lm_cache_.end();)
        {
            if (it->second.last_seen_kf < cutoff)
                it = lm_cache_.erase(it);
            else
                ++it;
        }

        // If still too big due to ties, erase arbitrary extras
        while (lm_cache_.size() > lm_cache_max_)
        {
            lm_cache_.erase(lm_cache_.begin());
        }
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

        cfg_.rig = rig;

        {
            std::lock_guard<std::mutex> lk(opt_mtx_);
            optimizer_ = std::make_shared<Optimizer>(cfg_);
        }
        rig_ready_ = true;

        RCLCPP_INFO(get_logger(),
                    "Stereo rig ready: fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.5f (m). Optimizer initialized.",
                    rig.left.fx(), rig.left.fy(), rig.left.cx(), rig.left.cy(), rig.baseline);
    }

    bool maybeResolveBodyCameraExtrinsic_()
    {
        if (!auto_resolve_t_bc_from_tf_ || t_bc_resolved_from_tf_)
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
                body_frame_id_, camera_frame, tf2::TimePointZero);
            cfg_.T_BC = transformMsgToIso(tf.transform);
            t_bc_resolved_from_tf_ = true;

            Eigen::Quaterniond q(cfg_.T_BC.rotation());
            q.normalize();
            RCLCPP_INFO(
                get_logger(),
                "Resolved T_BC from TF: %s <- %s, t=[%.6f %.6f %.6f], q_xyzw=[%.6f %.6f %.6f %.6f]",
                body_frame_id_.c_str(), camera_frame.c_str(),
                cfg_.T_BC.translation().x(), cfg_.T_BC.translation().y(), cfg_.T_BC.translation().z(),
                q.x(), q.y(), q.z(), q.w());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for TF %s <- %s to resolve T_BC: %s",
                body_frame_id_.c_str(), camera_frame.c_str(), ex.what());
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

            const auto ev = toKeyframeEvent(msg);
            if (ev.ids.empty())
                continue;

            std::optional<Eigen::Isometry3d> between_meas = std::nullopt;
            if (ev.has_vo_between)
            {
                between_meas = ev.T_Bkm1_Bk;
            }

            const auto optimize_start = std::chrono::steady_clock::now();
            const auto res = opt->push(ev, between_meas);
            const auto optimize_end = std::chrono::steady_clock::now();
            const double optimize_ms =
                std::chrono::duration<double, std::milli>(optimize_end - optimize_start).count();
            if (!res)
                continue;

            RCLCPP_INFO(
                get_logger(),
                "Optimization update kf_id=%lu took %.1f ms (window=%d stereo_factors=%d between_factors=%d)",
                static_cast<unsigned long>(res->kf_id),
                optimize_ms,
                res->stats.num_keyframes_in_window,
                res->stats.num_stereo_factors_added,
                res->stats.num_between_factors_added);

            // ---- Update persistent landmark cache ----
            {
                // Get current smoother landmark estimates (alive in window right now)
                const auto lms = opt->getLandmarks(lm_fetch_max_);
                mergeLandmarksIntoCache_(lms);
            }

            // publishing imu bias
            visual_inertial::msg::ImuBias bmsg;
            bmsg.header.stamp = msg.header.stamp; // same stamp as keyframe
            bmsg.header.frame_id = "base_link";   // or your body frame name
            bmsg.kf_id = res->kf_id;

            bmsg.accel_bias.x = res->bias_opt.accel.x();
            bmsg.accel_bias.y = res->bias_opt.accel.y();
            bmsg.accel_bias.z = res->bias_opt.accel.z();

            bmsg.gyro_bias.x = res->bias_opt.gyro.x();
            bmsg.gyro_bias.y = res->bias_opt.gyro.y();
            bmsg.gyro_bias.z = res->bias_opt.gyro.z();

            bias_pub_->publish(bmsg);

            visual_inertial::msg::OptimizationResult opt_msg;
            opt_msg.header.stamp = msg.header.stamp;
            opt_msg.header.frame_id = map_frame_id_;
            opt_msg.kf_id = res->kf_id;
            opt_msg.t_s = res->t_s;
            opt_msg.pose_wc_opt = isoToPoseMsg(res->T_WC_opt);
            opt_msg.pose_wb_opt = isoToPoseMsg(res->T_WB_opt);
            opt_msg.stats.num_keyframes_in_window = res->stats.num_keyframes_in_window;
            opt_msg.stats.num_landmarks_alive = res->stats.num_landmarks_alive;
            opt_msg.stats.num_landmarks_created = res->stats.num_landmarks_created;
            opt_msg.stats.num_stereo_factors_added = res->stats.num_stereo_factors_added;
            opt_msg.stats.num_between_factors_added = res->stats.num_between_factors_added;
            opt_msg.stats.num_prior_factors_added = res->stats.num_prior_factors_added;
            opt_msg.stats.final_error = res->stats.final_error;
            opt_msg.accel_bias.x = res->bias_opt.accel.x();
            opt_msg.accel_bias.y = res->bias_opt.accel.y();
            opt_msg.accel_bias.z = res->bias_opt.accel.z();
            opt_msg.gyro_bias.x = res->bias_opt.gyro.x();
            opt_msg.gyro_bias.y = res->bias_opt.gyro.y();
            opt_msg.gyro_bias.z = res->bias_opt.gyro.z();
            if (optimization_result_pub_)
                optimization_result_pub_->publish(opt_msg);

            // Update cached map->odom correction (timer will broadcast it at constant rate)
            // const Eigen::Isometry3d T_odom_C = poseMsgToIso(msg.pose_odom_body);
            // const Eigen::Isometry3d T_map_C = res->T_WC_opt;
            // const Eigen::Isometry3d T_odom_B = poseMsgToIso(msg.pose_odom_body); // now odom<-body
            // const Eigen::Isometry3d T_map_B = res->T_WB_opt;

            // const Eigen::Isometry3d T_map_odom = T_map_B * T_odom_B.inverse();
            Eigen::Isometry3d T_odom_B = poseMsgToIso(msg.pose_odom_body); // odom<-body (from KF msg)
            Eigen::Isometry3d T_map_B = res->T_WB_opt;              // map<-body (optimizer)

            Eigen::Isometry3d T_map_odom = T_map_B * T_odom_B.inverse();

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

            have_map_odom_.store(true);
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
                dt = 1.0 / publish_tf_hz_;
            last_filt_update_ = now;

            double alpha = dt / (smooth_tau_s_ + dt); // EMA equivalent
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
        auto tf = isoToTf(T_filt, now, map_frame_id_, odom_frame_id_);
        tf_broadcaster_->sendTransform(tf);
    }

    // Add this method to OptimizationNode (private:)
    void publishLandmarksPointCloud_()
    {
        if (!lm_pub_)
            return;

        // Snapshot cache under lock
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(lm_cache_.size());
        {
            std::lock_guard<std::mutex> lk(lm_mtx_);
            pts.reserve(lm_cache_.size());
            for (const auto &kv : lm_cache_)
            {
                pts.push_back(kv.second.p_map);
            }
        }

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_id_; // usually "map"
        msg.height = 1;
        msg.width = static_cast<uint32_t>(pts.size());
        msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(pts.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

        for (const auto &p : pts)
        {
            *it_x = static_cast<float>(p.x());
            *it_y = static_cast<float>(p.y());
            *it_z = static_cast<float>(p.z());
            ++it_x;
            ++it_y;
            ++it_z;
        }

        lm_pub_->publish(msg);
    }

private:
    // Topics / frames
    std::string keyframe_topic_;
    std::string left_info_topic_;
    std::string right_info_topic_;
    std::string body_frame_id_;
    std::string map_frame_id_;
    std::string odom_frame_id_;

    double tf_pub_rate_hz_{30.0};
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Subscriptions
    rclcpp::Subscription<visual_inertial::msg::Keyframe>::SharedPtr kf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool auto_resolve_t_bc_from_tf_{true};
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
    OptimizationConfig cfg_;
    std::mutex opt_mtx_;
    std::shared_ptr<Optimizer> optimizer_;

    // Queue / threading
    std::mutex q_mtx_;
    std::condition_variable q_cv_;
    std::deque<visual_inertial::msg::Keyframe> kf_q_;
    size_t max_queue_{30};

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

    double smooth_tau_s_ = 0.5; // time constant (tune 0.2..2.0)
    double publish_tf_hz_ = 30.0;

    rclcpp::Time last_filt_update_;

    // landmark cache
    // --- Landmark cache (persistent across window) ---
    std::mutex lm_mtx_;
    struct CachedLm
    {
        Eigen::Vector3d p_map = Eigen::Vector3d::Zero();
        uint64_t last_seen_kf = 0;
    };
    std::unordered_map<uint32_t, CachedLm> lm_cache_; // tid -> cached point
    size_t lm_cache_max_{2000};
    size_t lm_fetch_max_{0}; // 0 = fetch all currently-available landmarks

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lm_pub_;
    rclcpp::TimerBase::SharedPtr lm_timer_;
    bool publish_optimization_result_{true};
    std::string optimization_result_topic_{"optimization_result"};
    bool publish_optimized_landmarks_{true};
    std::string landmark_topic_{"optimized_landmarks"};
    double lm_pub_hz_{2.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptimizationNode>());
    rclcpp::shutdown();
    return 0;
}
