#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "visual_inertial_common/types.hpp"

struct OptimizationStats {
  // Window bookkeeping (requested window size, not necessarily exact internal marginal set)
  int num_keyframes_in_window = 0;

  // Landmark / factor counts for the *latest update() call*
  int num_landmarks_alive = 0;
  int num_landmarks_created = 0;

  int num_stereo_factors_added = 0;
  int num_between_factors_added = 0;
  int num_prior_factors_added = 0;

  // Optional: fill later if you want (requires querying smoother internals / error)
  double final_error = -1.0;
};

struct OptimizationResult {
  uint64_t kf_id = 0;
  double t_s = 0.0;

  // Optimized pose of left camera in world coordinates (World <- Camera)
  Eigen::Isometry3d T_WC_opt = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d T_WB_opt = Eigen::Isometry3d::Identity();

  OptimizationStats stats;
};

