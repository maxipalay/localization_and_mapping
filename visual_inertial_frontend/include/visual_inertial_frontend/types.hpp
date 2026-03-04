#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "visual_inertial_common/types.hpp"

struct FrameResult
{
    Eigen::Isometry3d vo_pose_abs = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d vo_pose_rel = Eigen::Isometry3d::Identity();

    bool kf_trigger = false;
    double stamp;
    
    cv::Mat debug_viz;
};

// IMU sample in IMU frame (or "body").
// gyro: rad/s, accel: m/s^2, timestamps in seconds (same clock domain as camera stamps).
struct ImuSample
{
    double t_s = 0.0;
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};