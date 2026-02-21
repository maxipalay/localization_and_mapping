#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

struct Camera
{
    int cam_id = 0;
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    Eigen::VectorXd D;

    float fx() const { return static_cast<float>(K(0, 0)); }
    float fy() const { return static_cast<float>(K(1, 1)); }
    float cx() const { return static_cast<float>(K(0, 2)); }
    float cy() const { return static_cast<float>(K(1, 2)); }
};

struct CameraRig
{
    Camera left;
    Camera right;
    double baseline{0.0};

    bool valid() const
    {
        return left.fx() > 0 && left.fy() > 0 && right.fx() > 0 && right.fy() > 0 && baseline > 0.0;
    }

    const Camera *get(int cam_id) const
    {
        if (cam_id == left.cam_id)
            return &left;
        if (cam_id == right.cam_id)
            return &right;
        return nullptr;
    }
};

struct KeyframeEvent
{
    using TrackId = uint32_t;

    // Identity / timing
    uint64_t kf_id = 0;
    double t_start = 0.0;
    double t_end = 0.0;

    // Pose of the LEFT camera in world coordinates (World <- Camera)
    Eigen::Isometry3d T_WC = Eigen::Isometry3d::Identity();

    // Tracked features for this keyframe (all vectors aligned by index)
    // ids[i] corresponds to pl[i] and (if available) pr[i].
    std::vector<TrackId> ids;
    std::vector<cv::Point2f> pl; // left image coords (pixels)
    std::vector<cv::Point2f> pr; // right image coords (pixels) (same ordering)
    std::vector<uint8_t> has_r;  // optional: 1 if pr[i] is valid, else 0

    bool has_imu = false;
    std::vector<uint8_t> pim_bytes;
};

// Bias estimate (gyro + accel), units: rad/s and m/s^2
struct ImuBias
{
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

// Optional “backend feedback” bundle.
// If you later want frontend propagation to be consistent, this is what you’d feed back.
struct ViState
{
    double t_s = 0.0;
    Eigen::Isometry3d T_WB = Eigen::Isometry3d::Identity();
    Eigen::Vector3d v_WB = Eigen::Vector3d::Zero();
    ImuBias bias;
    bool valid = false;
};

// What the frontend will attach to a keyframe message when you move preintegration to frontend.
// bytes contains a boost-serialized gtsam::PreintegratedCombinedMeasurements.

struct PreintegratedImuPacket
{
    uint64_t kf_id0 = 0;
    uint64_t kf_id1 = 0;

    double t0_s = 0.0; // interval start (camera time)
    double t1_s = 0.0; // interval end   (camera time)
    double dt_s = 0.0;

    ImuBias bias_hat;                  // bias linearization point used during integration
    uint32_t num_samples = 0;          // samples used in this interval
    uint32_t num_gaps = 0;             // dt gaps detected (diagnostic)
    uint32_t dropped_out_of_order = 0; // diagnostic

    std::vector<uint8_t> bytes; // boost-serialized GTSAM PIM
    bool valid = false;
};
