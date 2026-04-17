#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "visual_inertial_common/types.hpp"

struct FrontendHealth
{
    static constexpr uint8_t STATE_UNKNOWN = 0;
    static constexpr uint8_t STATE_TRACKING = 1;
    static constexpr uint8_t STATE_DEGRADED = 2;
    static constexpr uint8_t STATE_LOST = 3;

    int num_tracks = 0;
    int num_stereo_tracks = 0;
    int num_pnp_candidates = 0;
    int num_pnp_inliers = 0;

    double pnp_inlier_ratio = 0.0;
    double pnp_reproj_rmse_px = -1.0;
    double track_coverage = 0.0;

    bool pose_update_valid = false;
    uint8_t state = STATE_UNKNOWN;
};

struct FrameResult
{
    Eigen::Isometry3d vo_pose_abs = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d vo_pose_rel = Eigen::Isometry3d::Identity();

    bool kf_trigger = false;
    double stamp = 0.0;

    std::vector<cv::Point2f> tracks; // left camera tracks
    FrontendHealth health;
};

// IMU sample in IMU frame (or "body").
// gyro: rad/s, accel: m/s^2, timestamps in seconds (same clock domain as camera stamps).
struct ImuSample
{
    double t_s = 0.0;
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};
