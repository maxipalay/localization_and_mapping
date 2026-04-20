#pragma once

#include <array>

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
  int num_imu_factors_added = 0;
  int num_between_factors_added = 0;
  int num_prior_factors_added = 0;

  bool had_vo_between_measurement = false;
  bool used_vo_between_factor = false;
  bool skipped_vo_between_factor = false;
  double vo_between_quality = 1.0;
  double vo_between_sigma_scale = 1.0;

  bool imu_only_update = false;

  int update_iterations = 0;
  int update_intermediate_steps = 0;
  int update_nonlinear_variables = 0;
  int update_linear_variables = 0;

  double final_error = -1.0;
  bool has_error_before = false;
  double error_before = -1.0;
  bool has_error_after = false;
  double error_after = -1.0;

  int variables_relinearized = 0;
  int variables_reeliminated = 0;
  int factors_recalculated = 0;
  int cliques = 0;
};

struct OptimizationResult {
  uint64_t kf_id = 0;
  double t_s = 0.0;

  // Optimized pose of left camera in world coordinates (World <- Camera)
  Eigen::Isometry3d T_WC_opt = Eigen::Isometry3d::Identity();

  Eigen::Isometry3d T_WB_opt = Eigen::Isometry3d::Identity();

  OptimizationStats stats;

  ImuBias bias_opt; // accel+gyro
  Eigen::Vector3d velocity_opt = Eigen::Vector3d::Zero();
  bool has_velocity = false;

  bool has_pose_wb_covariance = false;
  std::array<double, 36> pose_wb_covariance{};

  bool has_velocity_covariance = false;
  std::array<double, 9> velocity_covariance{};

  bool has_bias_covariance = false;
  std::array<double, 36> bias_covariance{};
};
