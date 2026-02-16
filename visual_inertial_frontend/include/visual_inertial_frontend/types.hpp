#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "visual_inertial_common/types.hpp"

struct FrameResult
{
    Eigen::Isometry3d vo_pose_abs = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d vo_pose_rel = Eigen::Isometry3d::Identity();

    bool kf_valid = false;
    KeyframeEvent kf;

    cv::Mat debug_viz;
};
