#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>


struct FrameResult
{
    Eigen::Isometry3d vo_pose_abs = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d vo_pose_rel = Eigen::Isometry3d::Identity();
};
