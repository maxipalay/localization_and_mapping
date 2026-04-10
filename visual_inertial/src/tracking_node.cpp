#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sstream>
#include <iomanip>
#include <limits>
#include <vector>

#include "visual_inertial_frontend/types.hpp"
#include "visual_inertial_frontend/odometry.hpp"

#include "visual_inertial/msg/keyframe.hpp"
#include <std_msgs/msg/header.hpp>

#include <image_transport/subscriber_filter.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <Eigen/Geometry>

#include <visual_inertial/msg/imu_bias.hpp>
#include <visual_inertial/msg/tracks.hpp>

static inline geometry_msgs::msg::Quaternion R_to_quat(const cv::Matx33d &R)
{
    geometry_msgs::msg::Quaternion q;
    double tr = R(0, 0) + R(1, 1) + R(2, 2);
    double qw, qx, qy, qz;
    if (tr > 0)
    {
        double s = std::sqrt(tr + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (R(2, 1) - R(1, 2)) / s;
        qy = (R(0, 2) - R(2, 0)) / s;
        qz = (R(1, 0) - R(0, 1)) / s;
    }
    else if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2)))
    {
        double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
        qw = (R(2, 1) - R(1, 2)) / s;
        qx = 0.25 * s;
        qy = (R(0, 1) + R(1, 0)) / s;
        qz = (R(0, 2) + R(2, 0)) / s;
    }
    else if (R(1, 1) > R(2, 2))
    {
        double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
        qw = (R(0, 2) - R(2, 0)) / s;
        qx = (R(0, 1) + R(1, 0)) / s;
        qy = 0.25 * s;
        qz = (R(1, 2) + R(2, 1)) / s;
    }
    else
    {
        double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
        qw = (R(1, 0) - R(0, 1)) / s;
        qx = (R(0, 2) + R(2, 0)) / s;
        qy = (R(1, 2) + R(2, 1)) / s;
        qz = 0.25 * s;
    }
    double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    q.w = qw / n;
    q.x = qx / n;
    q.y = qy / n;
    q.z = qz / n;
    return q;
}

inline double rotationAngleRad(const cv::Matx33d &R)
{
    double tr = R(0, 0) + R(1, 1) + R(2, 2);
    double c = (tr - 1.0) * 0.5;
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c);
}

static inline builtin_interfaces::msg::Time toStamp(double tsec)
{
    builtin_interfaces::msg::Time s{};
    s.sec = static_cast<int32_t>(tsec);
    s.nanosec = static_cast<uint32_t>((tsec - s.sec) * 1e9);
    return s;
}

static inline Eigen::Isometry3d transformMsgToIso(const geometry_msgs::msg::Transform &t)
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

class FeatureNode : public rclcpp::Node
{
public:
    FeatureNode()
        : Node("visual_inertial")
    {
        // Params
        input_topic_left_ = declare_parameter<std::string>("input_topic_left", "/oak/left/image_rect");
        input_topic_right_ = declare_parameter<std::string>("input_topic_right", "/oak/right/image_rect");
        input_topic_imu_ = declare_parameter<std::string>("input_topic_imu", "/oak/imu");
        left_info_t = declare_parameter<std::string>("left/camera_info", "/oak/left/camera_info");
        right_info_t = declare_parameter<std::string>("right/camera_info", "/oak/right/camera_info");
        tracks_topic_ = declare_parameter<std::string>("tracks_topic", "/tracks");

        transport_ = declare_parameter<std::string>("transport", "compressed"); // or "raw"

        int queue = declare_parameter<int>("queue_size", 3);

        // NEW: approximate sync tuning
        double stereo_slop_s = declare_parameter<double>("stereo_slop_s", 0.01); // 10ms default
        double age_penalty = declare_parameter<double>("age_penalty", 0.0);      // 0 = prefer closest stamps

        // Publishers
        tracks_pub_ = create_publisher<visual_inertial::msg::Tracks>(tracks_topic_, rclcpp::SensorDataQoS());

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // (optional) let users override frame ids
        parent_frame_ = this->declare_parameter<std::string>("parent_frame_id", "odom");
        published_child_frame_ = this->declare_parameter<std::string>("child_frame_id", "body");
        body_frame_ = this->declare_parameter<std::string>("body_frame_id", published_child_frame_);
        auto_resolve_t_bc_from_tf_ =
            this->declare_parameter<bool>("auto_resolve_t_bc_from_tf", true);

        // Build VisualInertial params from ROS parameters
        VisualInertial::Params vi_params;

        vi_params.target_features = static_cast<uint16_t>(declare_parameter<int>(
            "target_features", static_cast<int>(vi_params.target_features)));
        vi_params.stereo_epi_eps_y = declare_parameter<double>(
            "stereo_epi_eps_y", vi_params.stereo_epi_eps_y);
        vi_params.stereo_disp_min = declare_parameter<double>(
            "stereo_disp_min", vi_params.stereo_disp_min);
        vi_params.stereo_disp_max = declare_parameter<double>(
            "stereo_disp_max", vi_params.stereo_disp_max);
        vi_params.fb_thr2 = declare_parameter<double>("fb_thr2", vi_params.fb_thr2);

        vi_params.imu_coverage_margin_s = declare_parameter<double>(
            "imu_coverage_margin_s", vi_params.imu_coverage_margin_s);
        vi_params.kf_ready_queue_max = static_cast<size_t>(declare_parameter<int>(
            "kf_ready_queue_max", static_cast<int>(vi_params.kf_ready_queue_max)));
        vi_params.kf_pending_queue_max = static_cast<size_t>(declare_parameter<int>(
            "kf_pending_queue_max", static_cast<int>(vi_params.kf_pending_queue_max)));
        vi_params.mask_scale = declare_parameter<int>("mask_scale", vi_params.mask_scale);
        vi_params.topup_grid_scale = declare_parameter<int>("topup_grid_scale", vi_params.topup_grid_scale);
        vi_params.max_total_tracks = static_cast<uint16_t>(declare_parameter<int>(
            "max_total_tracks", static_cast<int>(vi_params.max_total_tracks)));
        vi_params.topup_burst_factor = declare_parameter<double>(
            "topup_burst_factor", vi_params.topup_burst_factor);
        vi_params.topup_detect_factor = declare_parameter<double>(
            "topup_detect_factor", vi_params.topup_detect_factor);
        vi_params.pnp_iterations_count = declare_parameter<int>(
            "pnp_iterations_count", vi_params.pnp_iterations_count);
        vi_params.pnp_reproj_error_px = static_cast<float>(declare_parameter<double>(
            "pnp_reproj_error_px", static_cast<double>(vi_params.pnp_reproj_error_px)));
        vi_params.pnp_confidence = declare_parameter<double>(
            "pnp_confidence", vi_params.pnp_confidence);
        vi_params.pnp_refine_max_iters = declare_parameter<int>(
            "pnp_refine_max_iters", vi_params.pnp_refine_max_iters);
        vi_params.pnp_refine_eps = declare_parameter<double>(
            "pnp_refine_eps", vi_params.pnp_refine_eps);

        // Keyframe policy tuning
        auto &kfcfg = vi_params.kf_policy_cfg;
        kfcfg.min_kf_dt_s = declare_parameter<double>("kf_min_dt_s", kfcfg.min_kf_dt_s);
        kfcfg.max_kf_dt_s = declare_parameter<double>("kf_max_dt_s", kfcfg.max_kf_dt_s);
        kfcfg.min_trans_m = declare_parameter<double>("kf_min_trans_m", kfcfg.min_trans_m);
        kfcfg.min_rot_deg = declare_parameter<double>("kf_min_rot_deg", kfcfg.min_rot_deg);
        kfcfg.min_tracks = declare_parameter<int>("kf_min_tracks", kfcfg.min_tracks);
        kfcfg.min_pnp_tracks = declare_parameter<int>("kf_min_pnp_tracks", kfcfg.min_pnp_tracks);
        kfcfg.min_shared_tracks = declare_parameter<int>("kf_min_shared_tracks", kfcfg.min_shared_tracks);
        kfcfg.min_shared_ratio = declare_parameter<double>("kf_min_shared_ratio", kfcfg.min_shared_ratio);
        kfcfg.force_kf_on_max_interval = declare_parameter<bool>("kf_force_on_max_interval", kfcfg.force_kf_on_max_interval);
        kfcfg.allow_early_kf_on_quality_drop = declare_parameter<bool>("kf_allow_early_on_quality_drop", kfcfg.allow_early_kf_on_quality_drop);

        // IMU preintegration params
        auto &imucfg = vi_params.imu_cfg;
        imucfg.gyro_noise_density = declare_parameter<double>("imu_gyro_noise_density", imucfg.gyro_noise_density);
        imucfg.accel_noise_density = declare_parameter<double>("imu_accel_noise_density", imucfg.accel_noise_density);
        imucfg.gyro_bias_rw = declare_parameter<double>("imu_gyro_bias_rw", imucfg.gyro_bias_rw);
        imucfg.accel_bias_rw = declare_parameter<double>("imu_accel_bias_rw", imucfg.accel_bias_rw);
        imucfg.gravity_mps2 = declare_parameter<double>("imu_gravity_mps2", imucfg.gravity_mps2);
        imucfg.max_buffer_s = declare_parameter<double>("imu_max_buffer_s", imucfg.max_buffer_s);
        imucfg.keep_anchor = declare_parameter<bool>("imu_keep_anchor", imucfg.keep_anchor);

        // Optional override for body<-camera extrinsics
        Eigen::Quaterniond q_default(vi_params.T_BC.rotation());
        q_default.normalize();

        auto t_bc = declare_parameter<std::vector<double>>(
            "T_BC_translation", {vi_params.T_BC.translation().x(),
                                   vi_params.T_BC.translation().y(),
                                   vi_params.T_BC.translation().z()});
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
                vi_params.T_BC = T;
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

        visual_inertial_ = std::make_unique<VisualInertial>(vi_params);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe with transport string + QoS (Humble API)
        sub_left_.subscribe(this, input_topic_left_, transport_, rmw_qos_profile_sensor_data);
        sub_right_.subscribe(this, input_topic_right_, transport_, rmw_qos_profile_sensor_data);

        left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            left_info_t,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(cam_info_mtx_);
                last_left_info_ = std::move(msg);
            });

        right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            right_info_t,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(cam_info_mtx_);
                last_right_info_ = std::move(msg);
            });

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue), sub_left_, sub_right_);

        // Apply slop/penalty on the policy
        auto policy = sync_->getPolicy(); // NOTE: returns a pointer in your Jazzy build
        policy->setMaxIntervalDuration(rclcpp::Duration::from_seconds(stereo_slop_s));
        policy->setAgePenalty(age_penalty);

        sync_->registerCallback(std::bind(&FeatureNode::cb, this,
                                          std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "Stereo: L='%s' R='%s' IMU='%s' transport='%s' image-sync: queue=%d slop=%.3fms age_penalty=%.3f",
                    input_topic_left_.c_str(), input_topic_right_.c_str(), input_topic_imu_.c_str(), transport_.c_str(),
                    queue, stereo_slop_s * 1e3, age_penalty);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .reliable()
                       .durability_volatile();

        kf_pub_ = this->create_publisher<visual_inertial::msg::Keyframe>("keyframes", qos);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic_imu_,
            rclcpp::SensorDataQoS(),
            std::bind(&FeatureNode::imuCallback_, this, std::placeholders::_1));

        stop_worker_.store(false);
        kf_worker_ = std::thread([this]()
                                 { this->kfWorkerLoop_(); });

        // bias subscription
        auto bias_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

        bias_sub_ = this->create_subscription<visual_inertial::msg::ImuBias>(
            "imu_bias", bias_qos,
            std::bind(&FeatureNode::biasCallback_, this, std::placeholders::_1));
    }

    ~FeatureNode() override
    {
        stop_worker_.store(true);
        kf_worker_cv_.notify_all();
        if (kf_worker_.joinable())
            kf_worker_.join();
    }

private:
    bool vo_calibrated_{false};

    // member:
    std::unique_ptr<VisualInertial> visual_inertial_;

    rclcpp::Publisher<visual_inertial::msg::Keyframe>::SharedPtr kf_pub_;
    rclcpp::Publisher<visual_inertial::msg::Tracks>::SharedPtr tracks_pub_;

    std::string input_topic_left_, input_topic_right_, transport_;
    std::string input_topic_imu_;
    std::string left_info_t, right_info_t;
    std::string tracks_topic_;

    // image subscribers
    image_transport::SubscriberFilter sub_left_{}, sub_right_{};
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
    std::mutex cam_info_mtx_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_left_info_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_right_info_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string parent_frame_;
    std::string published_child_frame_;
    std::string body_frame_;
    bool auto_resolve_t_bc_from_tf_{true};
    bool t_bc_resolved_from_tf_{false};
    bool imu_rotation_resolved_{false};
    std::string imu_measurement_frame_;
    Eigen::Matrix3d R_body_imu_measurement_ = Eigen::Matrix3d::Identity();
    bool publish_child_offset_resolved_{false};
    Eigen::Isometry3d T_B_P_ = Eigen::Isometry3d::Identity();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // bias updates
    rclcpp::Subscription<visual_inertial::msg::ImuBias>::SharedPtr bias_sub_;
    std::atomic<uint64_t> last_bias_kf_id_{0}; // ignore old bias updates

    // debug stats
    std::mutex stamp_mtx_;
    double last_img_stamp_s_ = 0.0;
    double last_imu_stamp_s_ = 0.0;
    bool have_last_img_stamp_ = false;
    bool have_last_imu_stamp_ = false;
    // debug stats -end

    std::thread kf_worker_;
    std::mutex kf_worker_mtx_;
    std::condition_variable kf_worker_cv_;
    std::atomic<bool> stop_worker_{false};

    void biasCallback_(const visual_inertial::msg::ImuBias::SharedPtr msg)
    {
        // Optional: only accept monotonic keyframe bias updates
        const uint64_t kf_id = msg->kf_id;
        const uint64_t prev = last_bias_kf_id_.load(std::memory_order_relaxed);
        if (kf_id <= prev)
            return;
        last_bias_kf_id_.store(kf_id, std::memory_order_relaxed);

        ImuBias b;
        b.accel = Eigen::Vector3d(msg->accel_bias.x, msg->accel_bias.y, msg->accel_bias.z);
        b.gyro = Eigen::Vector3d(msg->gyro_bias.x, msg->gyro_bias.y, msg->gyro_bias.z);

        visual_inertial_->setImuBias(b);
    }

    void cb(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr right_info;
        {
            std::lock_guard<std::mutex> lk(cam_info_mtx_);
            left_info = last_left_info_;
            right_info = last_right_info_;
        }
        if (!left_info || !right_info)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for left/right CameraInfo before processing stereo pairs");
            return;
        }

        if (!maybeResolveBodyCameraExtrinsic_(left_info->header.frame_id))
        {
            return;
        }
        if (!maybeResolvePublishedChildOffset_())
        {
            return;
        }

        const double tL = rclcpp::Time(left_msg->header.stamp).seconds();
        const double tR = rclcpp::Time(right_msg->header.stamp).seconds();
        const double dLR = std::abs(tL - tR);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[SYNC] tL=%.9f tR=%.9f |L-R|=%.6f ms", tL, tR, dLR * 1e3);

        auto t0 = std::chrono::steady_clock::now();

        const double t_img = rclcpp::Time(left_msg->header.stamp).seconds();

        // debug stats - end
        double t_imu = 0.0;
        bool have_imu = false;
        {
            std::lock_guard<std::mutex> lk(stamp_mtx_);
            last_img_stamp_s_ = t_img;
            have_last_img_stamp_ = true;

            have_imu = have_last_imu_stamp_;
            t_imu = last_imu_stamp_s_;
        }

        if (have_imu)
        {
            const double dt = t_img - t_imu;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[SYNCDBG] img=%.6f imu_last=%.6f (img-imu)=%.6f s",
                t_img, t_imu, dt);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[SYNCDBG] img=%.6f imu_last=NA", t_img);
        }
        // debug stats - end

        auto left_cv = cv_bridge::toCvShare(left_msg, left_msg->encoding);
        auto right_cv = cv_bridge::toCvShare(right_msg, right_msg->encoding);
        if (!left_cv || !right_cv || left_cv->image.empty() || right_cv->image.empty())
            return;

        // Make stereo calibration for VO (only once unless changed)
        if (!vo_calibrated_)
        {
            const auto model = makeStereoModel(*left_info, *right_info);
            visual_inertial_->setCalibration(model);
            vo_calibrated_ = true;

            logStereoCalibration_(*left_info, *right_info, model);

            // also log your runtime sync settings (handy when you launch with params)
            RCLCPP_INFO(this->get_logger(),
                        "[CALIB SET] transport='%s' topics: L='%s' R='%s' L_info='%s' R_info='%s'",
                        transport_.c_str(),
                        input_topic_left_.c_str(), input_topic_right_.c_str(),
                        left_info_t.c_str(), right_info_t.c_str());
        }
        // Convert ROS time to double seconds
        const rclcpp::Time ts = left_msg->header.stamp;
        const double t_curr = ts.seconds();

        auto result = visual_inertial_->processStereo(left_cv->image, right_cv->image, t_curr);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = left_msg->header.stamp;
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = published_child_frame_;
        const Eigen::Isometry3d T_pub = result.vo_pose_abs * T_B_P_;
        Eigen::Vector3d t = T_pub.translation();
        Eigen::Quaterniond q(T_pub.rotation());
        tf_msg.transform.translation.x = t.x();
        tf_msg.transform.translation.y = t.y();
        tf_msg.transform.translation.z = t.z();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_broadcaster_->sendTransform(tf_msg);

        tf_msg.header.stamp = left_msg->header.stamp;
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = "rel";
        const auto &T2 = result.vo_pose_rel;
        t = T2.translation();
        q = Eigen::Quaterniond(T2.rotation());
        tf_msg.transform.translation.x = t.x();
        tf_msg.transform.translation.y = t.y();
        tf_msg.transform.translation.z = t.z();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_broadcaster_->sendTransform(tf_msg);

        visual_inertial::msg::Tracks tracks_msg;
        tracks_msg.header.stamp = left_msg->header.stamp;

        const auto N = result.tracks.size();
        tracks_msg.u_l.resize(N);
        tracks_msg.v_l.resize(N);

        for (size_t i = 0; i < N; ++i)
        {
            tracks_msg.u_l[i] = result.tracks[i].x;
            tracks_msg.v_l[i] = result.tracks[i].y;
        }

        tracks_pub_->publish(tracks_msg);


        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Synced pair at t=%.3f",
                             rclcpp::Time(left_msg->header.stamp).seconds());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "imageCb wall: %.3f ms (%.1f FPS)", ms, 1000.0 / std::max(ms, 0.001));

        kf_worker_cv_.notify_one();
    }

    void kfWorkerLoop_()
    {
        while (rclcpp::ok() && !stop_worker_.load())
        {
            // Wait for new IMU or new frames/KFs
            {
                std::unique_lock<std::mutex> lk(kf_worker_mtx_);
                kf_worker_cv_.wait_for(lk, std::chrono::milliseconds(2),
                                       [&]
                                       { return stop_worker_.load(); });
            }
            if (stop_worker_.load())
                break;

            // Try to finalize as many as possible
            for (;;)
            {
                const bool did_one = visual_inertial_->tryFinalizeOne();

                // Drain ready queue every pass
                KeyframeEvent ev;
                while (visual_inertial_->tryPopFinalizedKeyframe(ev))
                {
                    publishKeyframe_(ev); // your existing msg conversion + publisher
                }

                if (!did_one)
                    break;
            }
        }
    }

    void publishKeyframe_(KeyframeEvent &ev)
    {
        visual_inertial::msg::Keyframe msg;
        msg.header.stamp = toStamp(ev.t_end);
        msg.header.frame_id = "odom";
        msg.kf_id = ev.kf_id;
        msg.t_start = ev.t_start;
        msg.t_end = ev.t_end;

        msg.pose_odom_body.position.x = ev.T_OB.translation().x();
        msg.pose_odom_body.position.y = ev.T_OB.translation().y();
        msg.pose_odom_body.position.z = ev.T_OB.translation().z();
        Eigen::Quaterniond q(ev.T_OB.rotation());
        msg.pose_odom_body.orientation.w = q.w();
        msg.pose_odom_body.orientation.x = q.x();
        msg.pose_odom_body.orientation.y = q.y();
        msg.pose_odom_body.orientation.z = q.z();

        msg.has_vo_between = ev.has_vo_between ? uint8_t{1} : uint8_t{0};
        const Eigen::Quaterniond q_between(ev.T_Bkm1_Bk.rotation());
        msg.between_pose_prev_curr.position.x = ev.T_Bkm1_Bk.translation().x();
        msg.between_pose_prev_curr.position.y = ev.T_Bkm1_Bk.translation().y();
        msg.between_pose_prev_curr.position.z = ev.T_Bkm1_Bk.translation().z();
        msg.between_pose_prev_curr.orientation.w = q_between.w();
        msg.between_pose_prev_curr.orientation.x = q_between.x();
        msg.between_pose_prev_curr.orientation.y = q_between.y();
        msg.between_pose_prev_curr.orientation.z = q_between.z();

        const auto N = ev.ids.size();
        msg.track_ids.resize(N);
        msg.u_l.resize(N);
        msg.v_l.resize(N);
        msg.u_r.resize(N);
        msg.v_r.resize(N);
        msg.has_right.resize(N);

        for (size_t i = 0; i < N; ++i)
        {
            msg.track_ids[i] = ev.ids[i];
            msg.u_l[i] = ev.pl[i].x;
            msg.v_l[i] = ev.pl[i].y;
            msg.u_r[i] = ev.pr[i].x;
            msg.v_r[i] = ev.pr[i].y;
            msg.has_right[i] = ev.has_r[i];
        }

        msg.has_imu = ev.has_imu ? uint8_t{1} : uint8_t{0};
        msg.pim_bytes = ev.pim_bytes;

        kf_pub_->publish(msg);
    }

    CameraRig makeStereoModel(
        const sensor_msgs::msg::CameraInfo &L, const sensor_msgs::msg::CameraInfo &R)
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

    void imuCallback_(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const double t_imu = rclcpp::Time(msg->header.stamp).seconds();
        if (!maybeResolveImuRotation_(msg->header.frame_id))
        {
            return;
        }

        ImuSample s;
        s.t_s = t_imu;
        const Eigen::Vector3d accel_meas(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
        const Eigen::Vector3d gyro_meas(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        s.accel = R_body_imu_measurement_ * accel_meas;
        s.gyro = R_body_imu_measurement_ * gyro_meas;

        visual_inertial_->processImu(s);

        // wake finalizer: new IMU data may satisfy coverage for pending keyframes
        // worker_.notify_one();
        kf_worker_cv_.notify_one();

        const double t_imu_raw = rclcpp::Time(msg->header.stamp).seconds();

        // debug stats
        double t_img = 0.0;
        bool have_img = false;
        {
            std::lock_guard<std::mutex> lk(stamp_mtx_);
            last_imu_stamp_s_ = t_imu_raw;
            have_last_imu_stamp_ = true;

            have_img = have_last_img_stamp_;
            t_img = last_img_stamp_s_;
        }

        if (have_img)
        {
            const double dt = t_img - t_imu_raw;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[SYNCDBG] imu=%.6f img_last=%.6f (img-imu)=%.6f s",
                t_imu_raw, t_img, dt);
        }
        // debug stats - end
    }

    bool maybeResolveBodyCameraExtrinsic_(const std::string &camera_frame)
    {
        if (!auto_resolve_t_bc_from_tf_ || t_bc_resolved_from_tf_)
        {
            return true;
        }
        if (camera_frame.empty())
        {
            return false;
        }
        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                body_frame_, camera_frame, tf2::TimePointZero);
            const Eigen::Isometry3d T_BC = transformMsgToIso(tf.transform);
            visual_inertial_->setBodyCameraExtrinsic(T_BC);
            t_bc_resolved_from_tf_ = true;

            Eigen::Quaterniond q(T_BC.rotation());
            q.normalize();
            RCLCPP_INFO(
                get_logger(),
                "Resolved T_BC from TF: %s <- %s, t=[%.6f %.6f %.6f], q_xyzw=[%.6f %.6f %.6f %.6f]",
                body_frame_.c_str(), camera_frame.c_str(),
                T_BC.translation().x(), T_BC.translation().y(), T_BC.translation().z(),
                q.x(), q.y(), q.z(), q.w());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for TF %s <- %s to resolve T_BC: %s",
                body_frame_.c_str(), camera_frame.c_str(), ex.what());
            return false;
        }
    }

    bool maybeResolveImuRotation_(const std::string &imu_frame)
    {
        if (imu_frame.empty())
        {
            return false;
        }
        if (imu_rotation_resolved_ && imu_measurement_frame_ == imu_frame)
        {
            return true;
        }
        if (imu_frame == body_frame_)
        {
            imu_measurement_frame_ = imu_frame;
            R_body_imu_measurement_.setIdentity();
            imu_rotation_resolved_ = true;
            return true;
        }

        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                body_frame_, imu_frame, tf2::TimePointZero);
            const Eigen::Isometry3d T_BI = transformMsgToIso(tf.transform);
            imu_measurement_frame_ = imu_frame;
            R_body_imu_measurement_ = T_BI.rotation();
            imu_rotation_resolved_ = true;

            Eigen::Quaterniond q(R_body_imu_measurement_);
            q.normalize();
            RCLCPP_INFO(
                get_logger(),
                "Resolved IMU rotation from TF: %s <- %s, q_xyzw=[%.6f %.6f %.6f %.6f]",
                body_frame_.c_str(), imu_frame.c_str(),
                q.x(), q.y(), q.z(), q.w());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for TF %s <- %s to rotate IMU samples: %s",
                body_frame_.c_str(), imu_frame.c_str(), ex.what());
            return false;
        }
    }

    bool maybeResolvePublishedChildOffset_()
    {
        if (publish_child_offset_resolved_)
        {
            return true;
        }
        if (published_child_frame_ == body_frame_)
        {
            T_B_P_.setIdentity();
            publish_child_offset_resolved_ = true;
            return true;
        }

        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                body_frame_, published_child_frame_, tf2::TimePointZero);
            T_B_P_ = transformMsgToIso(tf.transform);
            publish_child_offset_resolved_ = true;

            Eigen::Quaterniond q(T_B_P_.rotation());
            q.normalize();
            RCLCPP_INFO(
                get_logger(),
                "Resolved published child offset from TF: %s <- %s, t=[%.6f %.6f %.6f], q_xyzw=[%.6f %.6f %.6f %.6f]",
                body_frame_.c_str(), published_child_frame_.c_str(),
                T_B_P_.translation().x(), T_B_P_.translation().y(), T_B_P_.translation().z(),
                q.x(), q.y(), q.z(), q.w());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for TF %s <- %s to publish odom child frame: %s",
                body_frame_.c_str(), published_child_frame_.c_str(), ex.what());
            return false;
        }
    }

    void logStereoCalibration_(
        const sensor_msgs::msg::CameraInfo &L,
        const sensor_msgs::msg::CameraInfo &R,
        const CameraRig &rig)
    {
        auto fmtK = [](const Eigen::Matrix3d &K)
        {
            std::ostringstream ss;
            ss.setf(std::ios::fixed);
            ss << std::setprecision(6);
            ss << "[[" << K(0, 0) << ", " << K(0, 1) << ", " << K(0, 2) << "], "
               << "[" << K(1, 0) << ", " << K(1, 1) << ", " << K(1, 2) << "], "
               << "[" << K(2, 0) << ", " << K(2, 1) << ", " << K(2, 2) << "]]";
            return ss.str();
        };

        auto p3_or_nan = [](const sensor_msgs::msg::CameraInfo &ci) -> double
        {
            if (ci.p.size() == 12)
                return ci.p[3];
            return std::numeric_limits<double>::quiet_NaN();
        };

        const double fxL = rig.left.K(0, 0), fyL = rig.left.K(1, 1), cxL = rig.left.K(0, 2), cyL = rig.left.K(1, 2);
        const double fxR = rig.right.K(0, 0), fyR = rig.right.K(1, 1), cxR = rig.right.K(0, 2), cyR = rig.right.K(1, 2);

        RCLCPP_INFO(this->get_logger(),
                    "\n[CALIB SET]\n"
                    "  L frame_id: %s  w,h: %u,%u  model: %s  D size: %zu\n"
                    "  R frame_id: %s  w,h: %u,%u  model: %s  D size: %zu\n"
                    "  L K: fx=%.6f fy=%.6f cx=%.6f cy=%.6f\n"
                    "  R K: fx=%.6f fy=%.6f cx=%.6f cy=%.6f\n"
                    "  L Kmat=%s\n"
                    "  R Kmat=%s\n"
                    "  L P[3]=%.6f   R P[3]=%.6f   (Tx from R: -P3/fxR)\n"
                    "  Baseline (rig.baseline)=%.6f m\n",
                    L.header.frame_id.c_str(), L.width, L.height, L.distortion_model.c_str(), L.d.size(),
                    R.header.frame_id.c_str(), R.width, R.height, R.distortion_model.c_str(), R.d.size(),
                    fxL, fyL, cxL, cyL,
                    fxR, fyR, cxR, cyR,
                    fmtK(rig.left.K).c_str(),
                    fmtK(rig.right.K).c_str(),
                    p3_or_nan(L), p3_or_nan(R),
                    rig.baseline);

        // Also print full P vectors if you want (throttled-ish; only printed once anyway)
        {
            std::ostringstream ss;
            ss.setf(std::ios::fixed);
            ss << std::setprecision(6);
            ss << "  L P=[";
            for (size_t i = 0; i < L.p.size(); ++i)
            {
                ss << L.p[i] << (i + 1 < L.p.size() ? ", " : "");
            }
            ss << "]\n  R P=[";
            for (size_t i = 0; i < R.p.size(); ++i)
            {
                ss << R.p[i] << (i + 1 < R.p.size() ? ", " : "");
            }
            ss << "]";
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureNode>());
    rclcpp::shutdown();
    return 0;
}
