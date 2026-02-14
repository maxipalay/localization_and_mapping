#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>


struct FrameResult
{
    Eigen::Isometry3d vo_pose_abs = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d vo_pose_rel = Eigen::Isometry3d::Identity();

    cv::Mat debug_viz;   
};

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

