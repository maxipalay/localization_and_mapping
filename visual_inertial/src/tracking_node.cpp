#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "visual_inertial_frontend/types.hpp"
#include "visual_inertial_frontend/odometry.hpp"

#include "visual_inertial/msg/keyframe.hpp"
#include <std_msgs/msg/header.hpp>

#include <image_transport/subscriber_filter.hpp>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <Eigen/Geometry>

#include <Eigen/Geometry>

// Converts an SE(3) transform expressed in an "optical" frame to a ROS-style frame,
// preserving the same similarity transform you used:
//   R_out = R_ros_opt * R_in * R_opt_ros
//   t_out = R_ros_opt * t_in
//
// Convention: T maps points as p_out = T * p_in (i.e., "destination <- source").
// This applies a fixed change-of-basis on both rotation and translation.
inline Eigen::Isometry3d poseOpticalToRos(const Eigen::Isometry3d &T_in)
{
    // Same matrix as your cv::Matx33d
    static const Eigen::Matrix3d R_ros_opt =
        (Eigen::Matrix3d() << 0.0, 0.0, 1.0,
         -1.0, 0.0, 0.0,
         0.0, -1.0, 0.0)
            .finished();

    const Eigen::Matrix3d R_opt_ros = R_ros_opt.transpose();

    Eigen::Isometry3d T_out = Eigen::Isometry3d::Identity();
    T_out.linear() = R_ros_opt * T_in.linear() * R_opt_ros;
    T_out.translation() = R_ros_opt * T_in.translation();
    return T_out;
}

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

class FeatureNode : public rclcpp::Node
{
public:
    FeatureNode()
        : Node("cuda_gftt_node")
    {
        // Params
        input_topic_left_ = declare_parameter<std::string>("input_topic_left", "/oak/left/image_rect");
        input_topic_right_ = declare_parameter<std::string>("input_topic_right", "/oak/right/image_rect");
        left_info_t = declare_parameter<std::string>("left/camera_info", "/oak/left/camera_info");
        right_info_t = declare_parameter<std::string>("right/camera_info", "/oak/right/camera_info");

        transport_ = declare_parameter<std::string>("transport", "compressed"); // or "raw"
        output_topic_ = declare_parameter<std::string>("output_topic", "/camera/image_features");

        int queue = declare_parameter<int>("queue_size", 3);

        // Publishers

        it_pub_ = image_transport::create_publisher(
            this, output_topic_, rmw_qos_profile_sensor_data);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // (optional) let users override frame ids
        parent_frame_ = this->declare_parameter<std::string>("parent_frame_id", "odom");
        child_frame_ = this->declare_parameter<std::string>("child_frame_id", "body");

        visual_inertial_ = std::make_unique<VisualInertial>();

        // Subscribe with transport string + QoS (Humble API)
        sub_left_.subscribe(this, input_topic_left_, transport_, rmw_qos_profile_sensor_data);
        sub_right_.subscribe(this, input_topic_right_, transport_, rmw_qos_profile_sensor_data);

        // CameraInfo subscribers (plain message_filters::Subscriber)
        sub_left_info_.subscribe(this, left_info_t, rmw_qos_profile_sensor_data);
        sub_right_info_.subscribe(this, right_info_t, rmw_qos_profile_sensor_data);

        using SyncPolicy = message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
            sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue), sub_left_, sub_left_info_, sub_right_, sub_right_info_);
        sync_->registerCallback(std::bind(&FeatureNode::cb, this,
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3, std::placeholders::_4));

        RCLCPP_INFO(get_logger(), "Stereo: L='%s' R='%s' transport='%s'",
                    input_topic_left_.c_str(), input_topic_right_.c_str(), transport_.c_str());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .reliable()
                       .durability_volatile();

        kf_pub_ = this->create_publisher<visual_inertial::msg::Keyframe>("keyframes", qos);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/oak/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&FeatureNode::imuCallback_, this, std::placeholders::_1));
    }

private:
    bool vo_calibrated_{false};

    // params/state
    std::string output_topic_;

    // member:
    std::unique_ptr<VisualInertial> visual_inertial_;
    image_transport::Publisher it_pub_;

    rclcpp::Publisher<visual_inertial::msg::Keyframe>::SharedPtr kf_pub_;

    std::string input_topic_left_, input_topic_right_, transport_;
    std::string left_info_t, right_info_t;

    // image subscribers
    image_transport::SubscriberFilter sub_left_{}, sub_right_{};
    // CameraInfo subscribers (regular message_filters)
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_left_info_, sub_right_info_;

    // 4-way sync policy: (left_img, left_info, right_img, right_info)
    using SyncPolicy = message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string parent_frame_;
    std::string child_frame_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    double last_cam_stamp_s_ = 0.0;
    bool have_cam_stamp_ = false;

    bool have_imu_offset_ = false;
    double cam_minus_imu_offset_s_ = 0.0; // t_cam = t_imu + cam_minus_imu_offset_s_

    void cb(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr &left_info,
            const sensor_msgs::msg::Image::ConstSharedPtr &right_msg,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr &right_info)
    {
        auto t0 = std::chrono::steady_clock::now();

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
        }

        // Convert ROS time to double seconds
        const rclcpp::Time ts = left_msg->header.stamp;
        const double t_curr = ts.seconds();

        last_cam_stamp_s_ = t_curr; // seconds (same thing you use for keyframes)
        have_cam_stamp_ = true;

        auto result = visual_inertial_->processStereo(left_cv->image, right_cv->image, t_curr);

        if (!result.debug_viz.empty())
        {
            auto out_msg = cv_bridge::CvImage(left_msg->header, "bgr8", result.debug_viz).toImageMsg();
            it_pub_.publish(out_msg);
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = left_msg->header.stamp;
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = child_frame_;
        const auto &T = poseOpticalToRos(result.vo_pose_abs);
        Eigen::Vector3d t = T.translation();
        Eigen::Quaterniond q(T.rotation());
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
        const auto &T2 = poseOpticalToRos(result.vo_pose_rel);
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

        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Synced pair at t=%.3f",
                             rclcpp::Time(left_msg->header.stamp).seconds());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "imageCb wall: %.3f ms (%.1f FPS)", ms, 1000.0 / std::max(ms, 0.001));

        if (result.kf_valid)
        {
            visual_inertial::msg::Keyframe msg;
            msg.header.stamp = left_msg->header.stamp; // or rclcpp::Clock().now()
            msg.header.frame_id = "world";             // set your world frame
            msg.kf_id = result.kf.kf_id;
            msg.t_start = result.kf.t_start;
            msg.t_end = result.kf.t_end;

            msg.pose_wc.position.x = result.kf.T_WC.translation().x();
            msg.pose_wc.position.y = result.kf.T_WC.translation().y();
            msg.pose_wc.position.z = result.kf.T_WC.translation().z();
            Eigen::Quaterniond q(result.kf.T_WC.rotation());
            msg.pose_wc.orientation.w = q.w();
            msg.pose_wc.orientation.x = q.x();
            msg.pose_wc.orientation.y = q.y();
            msg.pose_wc.orientation.z = q.z();

            const auto N = result.kf.ids.size();
            msg.track_ids.resize(N);
            msg.u_l.resize(N);
            msg.v_l.resize(N);
            msg.u_r.resize(N);
            msg.v_r.resize(N);
            msg.has_right.resize(N);

            for (size_t i = 0; i < N; ++i)
            {
                msg.track_ids[i] = result.kf.ids[i];
                msg.u_l[i] = result.kf.pl[i].x;
                msg.v_l[i] = result.kf.pl[i].y;
                msg.u_r[i] = result.kf.pr[i].x;
                msg.v_r[i] = result.kf.pr[i].y;
                msg.has_right[i] = result.kf.has_r[i];
            }

            msg.has_imu = result.kf.has_imu ? uint8_t{1} : uint8_t{0};
            msg.pim_bytes = result.kf.pim_bytes;

            kf_pub_->publish(msg);
        }
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

        if (!have_imu_offset_ && have_cam_stamp_)
        {
            cam_minus_imu_offset_s_ = last_cam_stamp_s_ - t_imu;
            have_imu_offset_ = true;
            std::cout << std::fixed << std::setprecision(6)
                      << "[IMU] computed cam_minus_imu_offset_s_=" << cam_minus_imu_offset_s_
                      << " (cam=" << last_cam_stamp_s_ << " imu=" << t_imu << ")\n";
        }

        const double t_cam = have_imu_offset_ ? (t_imu + cam_minus_imu_offset_s_) : t_imu;

        ImuSample s;
        s.t_s = t_cam;
        s.accel = Eigen::Vector3d(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
        s.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);

        visual_inertial_->processImu(s);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureNode>());
    rclcpp::shutdown();
    return 0;
}
