// visual_inertial_optimization/src/optimization.cpp
#include "visual_inertial_optimization/optimization.hpp"

#include <cmath>
#include <optional>

#include <gtsam/linear/NoiseModel.h>

static inline gtsam::Point3 toGtsamPoint3_(const Eigen::Vector3d& p) {
  return gtsam::Point3(p.x(), p.y(), p.z());
}

static inline Eigen::Vector3d triangulateStereoInLeftCam_(
    double uL, double uR, double v,
    double fx, double fy, double cx, double cy,
    double baseline_m)
{
  const double disp = uL - uR;
  const double eps = 1e-9;
  const double d = (std::abs(disp) < eps) ? (disp >= 0 ? eps : -eps) : disp;

  const double Z = fx * baseline_m / d;
  const double X = (uL - cx) * Z / fx;
  const double Y = (v  - cy) * Z / fy;
  return Eigen::Vector3d(X, Y, Z);
}

std::optional<OptimizationResult> Optimizer::push(
    const KeyframeEvent& kf,
    const std::optional<Eigen::Isometry3d>& T_Ck_Ckm1)
{
  if (kf.kf_id == 0) return std::nullopt;
  if (!cfg_.rig.valid()) return std::nullopt;

  const size_t N = kf.ids.size();
  if (kf.pl.size() != N || kf.pr.size() != N || kf.has_r.size() != N) return std::nullopt;

  // kf_id is our fixed-lag "time"
  if (initialized_ && kf.kf_id <= last_kf_id_) return std::nullopt;

  // T_WB_init from incoming camera optical pose:
  // T_WC = T_WB * T_BC  =>  T_WB = T_WC * T_CB
  const Eigen::Isometry3d T_CB = cfg_.T_BC.inverse();
  const Eigen::Isometry3d T_WB_init = kf.T_WC * T_CB;

  // Optional bookkeeping prune (count-based window)
  if (cfg_.prune_unobserved_landmarks && cfg_.window_size > 0) {
    const uint64_t lag = cfg_.window_size - 1;
    const uint64_t min_alive = (kf.kf_id > lag) ? (kf.kf_id - lag) : 0;
    for (auto it = landmark_last_seen_kf_.begin(); it != landmark_last_seen_kf_.end(); ) {
      if (it->second < min_alive) it = landmark_last_seen_kf_.erase(it);
      else ++it;
    }
  }

  gtsam::NonlinearFactorGraph newFactors;
  gtsam::Values newValues;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;

  const double t_now = static_cast<double>(kf.kf_id);

  const gtsam::Key xk = X_(kf.kf_id);
  timestamps[xk] = t_now;

  // Insert BODY pose initial value
  newValues.insert(xk, toGtsamPose3_(T_WB_init));

  // Gauge prior on the first pose after reset (on BODY pose)
  int prior_added = 0;
  if (!initialized_) {
    const gtsam::Vector6 sigmas = (gtsam::Vector6() <<
        cfg_.prior_rot_sigma_rad, cfg_.prior_rot_sigma_rad, cfg_.prior_rot_sigma_rad,
        cfg_.prior_trans_sigma_m, cfg_.prior_trans_sigma_m, cfg_.prior_trans_sigma_m).finished();
    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(xk, toGtsamPose3_(T_WB_init), priorNoise));
    prior_added = 1;
  }

  // Optional VO BetweenFactor on BODY poses
  int between_added = 0;
  if (cfg_.use_vo_between && initialized_ && T_Ck_Ckm1.has_value()) {
    const gtsam::Key xkm1 = X_(last_kf_id_);

    // Convert camera relative (Ck <- Ck-1) to body relative (Bk <- Bk-1):
    // T_Bk_Bkm1 = T_BC * T_Ck_Ckm1 * T_CB
    const Eigen::Isometry3d T_Bk_Bkm1 = cfg_.T_BC * (*T_Ck_Ckm1) * T_CB;

    // BetweenFactor expects measurement = (Bkm1 <- Bk)
    const Eigen::Isometry3d T_Bkm1_Bk = T_Bk_Bkm1.inverse();
    const gtsam::Pose3 meas = toGtsamPose3_(T_Bkm1_Bk);

    const gtsam::Vector6 sigmas = (gtsam::Vector6() <<
        cfg_.between_rot_sigma_rad, cfg_.between_rot_sigma_rad, cfg_.between_rot_sigma_rad,
        cfg_.between_trans_sigma_m, cfg_.between_trans_sigma_m, cfg_.between_trans_sigma_m).finished();
    auto betweenNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    newFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(xkm1, xk, meas, betweenNoise));
    between_added = 1;
  }

  auto stereoNoise = gtsam::noiseModel::Isotropic::Sigma(3, cfg_.stereo_sigma_px);
  using StereoFactor = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;

  const auto& Lcam = cfg_.rig.left;
  const double fx = static_cast<double>(Lcam.fx());
  const double fy = static_cast<double>(Lcam.fy());
  const double cx = static_cast<double>(Lcam.cx());
  const double cy = static_cast<double>(Lcam.cy());
  const double b  = cfg_.rig.baseline;

  int stereo_factors_added = 0;
  int landmarks_created = 0;

  // Camera pose derived from body init (consistency)
  const Eigen::Isometry3d T_WC_from_body = T_WB_init * cfg_.T_BC;

  for (size_t i = 0; i < N; ++i) {
    if (kf.has_r[i] == 0) continue;

    const TrackId tid = kf.ids[i];
    const gtsam::Key lk = L_(tid);

    const bool new_landmark = (landmark_last_seen_kf_.find(tid) == landmark_last_seen_kf_.end());

    const double uL = static_cast<double>(kf.pl[i].x);
    const double v  = static_cast<double>(kf.pl[i].y);
    const double uR = static_cast<double>(kf.pr[i].x);

    const gtsam::StereoPoint2 z(uL, uR, v);

    // Important: pass body_T_cam_ so pose variable is BODY and sensor is CAMERA
    newFactors.add(StereoFactor(z, stereoNoise, xk, lk, Kst_, body_T_cam_));
    ++stereo_factors_added;

    if (new_landmark && cfg_.init_landmarks_from_stereo) {
      const double disp = uL - uR; // frontend should have gated; keep defensive check
      if (disp > 1e-9) {
        const Eigen::Vector3d pC = triangulateStereoInLeftCam_(uL, uR, v, fx, fy, cx, cy, b);
        const Eigen::Vector3d pW = T_WC_from_body * pC; // World <- CameraOptical
        newValues.insert(lk, toGtsamPoint3_(pW));
        ++landmarks_created;
      }
    }

    // Refresh landmark timestamp
    timestamps[lk] = t_now;
    landmark_last_seen_kf_[tid] = kf.kf_id;
  }

  try {
    smoother_.update(newFactors, newValues, timestamps);
  } catch (...) {
    return std::nullopt;
  }

  OptimizationResult out;
  out.kf_id = kf.kf_id;
  out.t_s   = kf.t_end;

  try {
    const gtsam::Pose3 T_WB_opt_g = smoother_.calculateEstimate<gtsam::Pose3>(xk);
    out.T_WB_opt = toEigenIsometry3d_(T_WB_opt_g);
    out.T_WC_opt = out.T_WB_opt * cfg_.T_BC;
  } catch (...) {
    return std::nullopt;
  }

  out.stats.num_keyframes_in_window = static_cast<int>(cfg_.window_size);
  out.stats.num_landmarks_alive = static_cast<int>(landmark_last_seen_kf_.size());
  out.stats.num_landmarks_created = landmarks_created;
  out.stats.num_stereo_factors_added = stereo_factors_added;
  out.stats.num_between_factors_added = between_added;
  out.stats.num_prior_factors_added = prior_added;

  initialized_ = true;
  last_kf_id_ = kf.kf_id;

  return out;
}


