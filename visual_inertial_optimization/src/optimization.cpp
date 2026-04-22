// //visual_inertial_optimization/src/optimization.cpp
// #include "visual_inertial_optimization/optimization.hpp"

// #include <cmath>
// #include <optional>

// #include <gtsam/linear/NoiseModel.h>

// static inline gtsam::Point3 toGtsamPoint3_(const Eigen::Vector3d& p) {
//   return gtsam::Point3(p.x(), p.y(), p.z());
// }

// static inline Eigen::Vector3d triangulateStereoInLeftCam_(
//     double uL, double uR, double v,
//     double fx, double fy, double cx, double cy,
//     double baseline_m)
// {
//   const double disp = uL - uR;
//   const double eps = 1e-9;
//   const double d = (std::abs(disp) < eps) ? (disp >= 0 ? eps : -eps) : disp;

//   const double Z = fx * baseline_m / d;
//   const double X = (uL - cx) * Z / fx;
//   const double Y = (v  - cy) * Z / fy;
//   return Eigen::Vector3d(X, Y, Z);
// }

// std::optional<OptimizationResult> Optimizer::push(
//     const KeyframeEvent& kf,
//     const std::optional<Eigen::Isometry3d>& T_Ck_Ckm1)
// {
//   if (kf.kf_id == 0) return std::nullopt;
//   if (!cfg_.rig.valid()) return std::nullopt;

//   const size_t N = kf.ids.size();
//   if (kf.pl.size() != N || kf.pr.size() != N || kf.has_r.size() != N) return std::nullopt;

//   // kf_id is our fixed-lag "time"
//   if (initialized_ && kf.kf_id <= last_kf_id_) return std::nullopt;

//   // T_WB_init from incoming camera optical pose:
//   // T_WC = T_WB * T_BC  =>  T_WB = T_WC * T_CB
//   const Eigen::Isometry3d T_CB = cfg_.T_BC.inverse();
//   const Eigen::Isometry3d T_WB_init = kf.T_WC * T_CB;

//   // Optional bookkeeping prune (count-based window)
//   if (cfg_.prune_unobserved_landmarks && cfg_.window_size > 0) {
//     const uint64_t lag = cfg_.window_size - 1;
//     const uint64_t min_alive = (kf.kf_id > lag) ? (kf.kf_id - lag) : 0;
//     for (auto it = landmark_last_seen_kf_.begin(); it != landmark_last_seen_kf_.end(); ) {
//       if (it->second < min_alive) it = landmark_last_seen_kf_.erase(it);
//       else ++it;
//     }
//   }

//   gtsam::NonlinearFactorGraph newFactors;
//   gtsam::Values newValues;
//   gtsam::FixedLagSmoother::KeyTimestampMap timestamps;

//   const double t_now = static_cast<double>(kf.kf_id);

//   const gtsam::Key xk = X_(kf.kf_id);
//   timestamps[xk] = t_now;

//   // Insert BODY pose initial value
//   newValues.insert(xk, toGtsamPose3_(T_WB_init));

//   // Gauge prior on the first pose after reset (on BODY pose)
//   int prior_added = 0;
//   if (!initialized_) {
//     const gtsam::Vector6 sigmas = (gtsam::Vector6() <<
//         cfg_.prior_rot_sigma_rad, cfg_.prior_rot_sigma_rad, cfg_.prior_rot_sigma_rad,
//         cfg_.prior_trans_sigma_m, cfg_.prior_trans_sigma_m, cfg_.prior_trans_sigma_m).finished();
//     auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
//     newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(xk, toGtsamPose3_(T_WB_init), priorNoise));
//     prior_added = 1;
//   }

//   // Optional VO BetweenFactor on BODY poses
//   int between_added = 0;
//   if (cfg_.use_vo_between && initialized_ && T_Ck_Ckm1.has_value()) {
//     const gtsam::Key xkm1 = X_(last_kf_id_);

//     // Convert camera relative (Ck <- Ck-1) to body relative (Bk <- Bk-1):
//     // T_Bk_Bkm1 = T_BC * T_Ck_Ckm1 * T_CB
//     const Eigen::Isometry3d T_Bk_Bkm1 = cfg_.T_BC * (*T_Ck_Ckm1) * T_CB;

//     // BetweenFactor expects measurement = (Bkm1 <- Bk)
//     const Eigen::Isometry3d T_Bkm1_Bk = T_Bk_Bkm1.inverse();
//     const gtsam::Pose3 meas = toGtsamPose3_(T_Bkm1_Bk);

//     const gtsam::Vector6 sigmas = (gtsam::Vector6() <<
//         cfg_.between_rot_sigma_rad, cfg_.between_rot_sigma_rad, cfg_.between_rot_sigma_rad,
//         cfg_.between_trans_sigma_m, cfg_.between_trans_sigma_m, cfg_.between_trans_sigma_m).finished();
//     auto betweenNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

//     newFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(xkm1, xk, meas, betweenNoise));
//     between_added = 1;
//   }

//   auto stereoNoise = gtsam::noiseModel::Isotropic::Sigma(3, cfg_.stereo_sigma_px);
//   using StereoFactor = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;

//   const auto& Lcam = cfg_.rig.left;
//   const double fx = static_cast<double>(Lcam.fx());
//   const double fy = static_cast<double>(Lcam.fy());
//   const double cx = static_cast<double>(Lcam.cx());
//   const double cy = static_cast<double>(Lcam.cy());
//   const double b  = cfg_.rig.baseline;

//   int stereo_factors_added = 0;
//   int landmarks_created = 0;

//   // Camera pose derived from body init (consistency)
//   const Eigen::Isometry3d T_WC_from_body = T_WB_init * cfg_.T_BC;

//   for (size_t i = 0; i < N; ++i) {
//     if (kf.has_r[i] == 0) continue;

//     const TrackId tid = kf.ids[i];
//     const gtsam::Key lk = L_(tid);

//     const bool new_landmark = (landmark_last_seen_kf_.find(tid) == landmark_last_seen_kf_.end());

//     const double uL = static_cast<double>(kf.pl[i].x);
//     const double v  = static_cast<double>(kf.pl[i].y);
//     const double uR = static_cast<double>(kf.pr[i].x);

//     const gtsam::StereoPoint2 z(uL, uR, v);

//     // Important: pass body_T_cam_ so pose variable is BODY and sensor is CAMERA
//     newFactors.add(StereoFactor(z, stereoNoise, xk, lk, Kst_, body_T_cam_));
//     ++stereo_factors_added;

//     if (new_landmark && cfg_.init_landmarks_from_stereo) {
//       const double disp = uL - uR; // frontend should have gated; keep defensive check
//       if (disp > 1e-9) {
//         const Eigen::Vector3d pC = triangulateStereoInLeftCam_(uL, uR, v, fx, fy, cx, cy, b);
//         const Eigen::Vector3d pW = T_WC_from_body * pC; // World <- CameraOptical
//         newValues.insert(lk, toGtsamPoint3_(pW));
//         ++landmarks_created;
//       }
//     }

//     // Refresh landmark timestamp
//     timestamps[lk] = t_now;
//     landmark_last_seen_kf_[tid] = kf.kf_id;
//   }

//   try {
//     smoother_.update(newFactors, newValues, timestamps);
//   } catch (...) {
//     return std::nullopt;
//   }

//   OptimizationResult out;
//   out.kf_id = kf.kf_id;
//   out.t_s   = kf.t_end;

//   try {
//     const gtsam::Pose3 T_WB_opt_g = smoother_.calculateEstimate<gtsam::Pose3>(xk);
//     out.T_WB_opt = toEigenIsometry3d_(T_WB_opt_g);
//     out.T_WC_opt = out.T_WB_opt * cfg_.T_BC;
//   } catch (...) {
//     return std::nullopt;
//   }

//   out.stats.num_keyframes_in_window = static_cast<int>(cfg_.window_size);
//   out.stats.num_landmarks_alive = static_cast<int>(landmark_last_seen_kf_.size());
//   out.stats.num_landmarks_created = landmarks_created;
//   out.stats.num_stereo_factors_added = stereo_factors_added;
//   out.stats.num_between_factors_added = between_added;
//   out.stats.num_prior_factors_added = prior_added;

//   initialized_ = true;
//   last_kf_id_ = kf.kf_id;

//   return out;
// }

// visual_inertial_optimization/src/optimization.cpp
#include "visual_inertial_optimization/optimization.hpp"

#include <cmath>
#include <array>
#include <optional>
#include <sstream>
#include <string>
#include <iostream>
#include <algorithm>

#include <gtsam/linear/NoiseModel.h>

// Deserialize IMU blob
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

std::vector<LandmarkEstimate> Optimizer::getLandmarks(size_t max_points) const
{
  std::lock_guard<std::mutex> lk(mtx_);

  std::vector<LandmarkEstimate> out;
  out.reserve(landmark_last_seen_kf_.size());

  for (const auto &kv : landmark_last_seen_kf_)
  {
    const TrackId tid = kv.first;
    const uint64_t last_seen = kv.second;
    const gtsam::Key lk_key = L_(tid);

    try
    {
      const gtsam::Point3 p = smoother_.calculateEstimate<gtsam::Point3>(lk_key);
      LandmarkEstimate e;
      e.tid = tid;
      e.p_W = Eigen::Vector3d(p.x(), p.y(), p.z());
      e.last_seen_kf = last_seen;
      out.push_back(e);
    }
    catch (...)
    {
      // Landmark might have been marginalized already; skip.
    }
  }

  if (max_points > 0 && out.size() > max_points)
  {
    // Keep most recent max_points by last_seen_kf
    std::nth_element(out.begin(), out.end() - max_points, out.end(),
                     [](const LandmarkEstimate &a, const LandmarkEstimate &b)
                     {
                       return a.last_seen_kf < b.last_seen_kf;
                     });
    out.erase(out.begin(), out.end() - max_points);
  }

  // Optional: sort newest->oldest for nicer behavior
  std::sort(out.begin(), out.end(),
            [](const LandmarkEstimate &a, const LandmarkEstimate &b)
            {
              return a.last_seen_kf > b.last_seen_kf;
            });

  return out;
}

static inline gtsam::Point3 toGtsamPoint3_(const Eigen::Vector3d &p)
{
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
  const double Y = (v - cy) * Z / fy;
  return Eigen::Vector3d(X, Y, Z);
}

static gtsam::SharedNoiseModel maybeHuberize_(
    const gtsam::SharedNoiseModel &base_noise,
    double huber_k)
{
  if (huber_k <= 0.0)
  {
    return base_noise;
  }
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
      base_noise);
}

static double clamp01_(double x)
{
  return std::clamp(x, 0.0, 1.0);
}

static double normalizedFloorScore_(double value, double min_acceptable)
{
  if (min_acceptable <= 0.0)
  {
    return 1.0;
  }
  return clamp01_(value / min_acceptable);
}

static double normalizedCeilingScore_(double value, double max_acceptable)
{
  if (value < 0.0 || max_acceptable <= 0.0)
  {
    return 1.0;
  }
  return clamp01_(max_acceptable / std::max(value, 1e-9));
}

static double computeVoBetweenIntervalQuality_(
    const FrontendIntervalHealth &health,
    const OptimizationConfig &cfg)
{
  if (health.num_frames == 0)
  {
    return 1.0;
  }

  const double num_frames = static_cast<double>(health.num_frames);
  const double pose_valid_fraction =
      static_cast<double>(health.num_pose_valid_frames) / num_frames;
  const double degraded_fraction =
      static_cast<double>(health.num_degraded_frames) / num_frames;
  const double lost_fraction =
      static_cast<double>(health.num_lost_frames) / num_frames;

  const double pose_valid_score =
      normalizedFloorScore_(pose_valid_fraction, cfg.between_health_min_pose_valid_fraction);
  const double retention_score =
      normalizedFloorScore_(health.min_track_retention, cfg.between_health_min_track_retention);
  const double pnp_score =
      normalizedFloorScore_(health.mean_pnp_inlier_ratio, cfg.between_health_min_pnp_inlier_ratio);
  const double coverage_score =
      normalizedFloorScore_(health.min_track_coverage, cfg.between_health_min_track_coverage);
  const double rmse_score =
      normalizedCeilingScore_(health.max_pnp_reproj_rmse_px, cfg.between_health_max_pnp_reproj_rmse_px);
  const double state_score = clamp01_(1.0 - std::max(degraded_fraction, lost_fraction));

  return std::min({pose_valid_score, retention_score, pnp_score, coverage_score, rmse_score, state_score});
}

static bool shouldSkipVoBetweenInterval_(
    const FrontendIntervalHealth &health,
    const OptimizationConfig &cfg,
    double interval_quality)
{
  if (health.num_frames == 0)
  {
    return false;
  }

  if (interval_quality >= cfg.between_health_skip_quality)
  {
    return false;
  }

  // Only skip when the interval is both low-quality and effectively has no
  // usable visual pose support. Otherwise keep a very loose between-factor.
  return health.num_pose_valid_frames == 0;
}

static bool deserializePim_(
    const std::vector<uint8_t> &bytes,
    gtsam::PreintegratedCombinedMeasurements *out)
{
  if (!out || bytes.empty())
    return false;
  try
  {
    std::string s(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    std::istringstream iss(s, std::ios::binary);
    boost::archive::binary_iarchive ia(iss);

    ia.register_type<gtsam::PreintegrationCombinedParams>();
    ia >> (*out);
    return true;
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] PIM deserialize exception: " << e.what() << std::endl;
    return false;
  }
  catch (...)
  {
    std::cout << "[Optimizer] PIM deserialize unknown exception\n";
    return false;
  }
}

template <size_t Rows, size_t Cols>
static std::array<double, Rows * Cols> toRowMajorArray_(const gtsam::Matrix &M)
{
  std::array<double, Rows * Cols> out{};
  for (size_t r = 0; r < Rows; ++r)
  {
    for (size_t c = 0; c < Cols; ++c)
    {
      out[r * Cols + c] = M(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
    }
  }
  return out;
}

std::optional<OptimizationResult> Optimizer::push(
    const KeyframeEvent &kf,
    const std::optional<Eigen::Isometry3d> &T_Bkm1_Bk_meas)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (kf.kf_id == 0)
    return std::nullopt;
  if (!cfg_.rig.valid())
    return std::nullopt;

  const size_t N = kf.ids.size();
  if (kf.pl.size() != N || kf.pr.size() != N || kf.has_r.size() != N)
    return std::nullopt;

  // kf_id is our fixed-lag "time"
  if (initialized_ && kf.kf_id <= last_kf_id_)
    return std::nullopt;

  // If IMU is enabled, require consecutive keyframes and valid IMU blob after the first.
  if (cfg_.use_imu && initialized_)
  {
    if (kf.kf_id != last_kf_id_ + 1)
    {
      std::cout << "[Optimizer] non-consecutive kf_id=" << kf.kf_id
                << " last=" << last_kf_id_ << "\n";
      return std::nullopt;
    }
    if (!kf.has_imu || kf.pim_bytes.empty())
    {
      std::cout << "[Optimizer] missing IMU on kf_id=" << kf.kf_id
                << " has_imu=" << int(kf.has_imu)
                << " bytes=" << kf.pim_bytes.size() << "\n";
      return std::nullopt;
    }
  }

  // T_WB_init from incoming camera optical pose:
  // Frontend-exported initialization pose in the odom/startup frame (odom <- body).
  const Eigen::Isometry3d T_OB_init = kf.T_OB;

  // Optional bookkeeping prune (count-based window)
  if (cfg_.prune_unobserved_landmarks && cfg_.window_size > 0)
  {
    const uint64_t lag = cfg_.window_size - 1;
    const uint64_t min_alive = (kf.kf_id > lag) ? (kf.kf_id - lag) : 0;
    for (auto it = landmark_last_seen_kf_.begin(); it != landmark_last_seen_kf_.end();)
    {
      if (it->second < min_alive)
        it = landmark_last_seen_kf_.erase(it);
      else
        ++it;
    }
  }

  gtsam::NonlinearFactorGraph newFactors;
  gtsam::Values newValues;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;

  const double t_now = static_cast<double>(kf.kf_id);

  const gtsam::Key xk = X_(kf.kf_id);
  const gtsam::Key vk = V_(kf.kf_id);
  const gtsam::Key bk = B_(kf.kf_id);

  timestamps[xk] = t_now;
  timestamps[vk] = t_now;
  timestamps[bk] = t_now;

  // Insert BODY pose initial value
  newValues.insert(xk, toGtsamPose3_(T_OB_init));

  // Insert velocity + bias initials
  gtsam::Vector3 v_init(0.0, 0.0, 0.0);
  gtsam::imuBias::ConstantBias b_init;

  if (initialized_)
  {
    try
    {
      v_init = smoother_.calculateEstimate<gtsam::Vector3>(V_(last_kf_id_));
    }
    catch (...)
    {
    }
    try
    {
      b_init = smoother_.calculateEstimate<gtsam::imuBias::ConstantBias>(B_(last_kf_id_));
    }
    catch (...)
    {
    }
  }

  newValues.insert(vk, v_init);
  newValues.insert(bk, b_init);

  // Priors on the first keyframe after reset
  int prior_added = 0;
  if (!initialized_)
  {
    // Pose prior
    const gtsam::Vector6 sigmas = (gtsam::Vector6() << cfg_.prior_rot_sigma_rad, cfg_.prior_rot_sigma_rad, cfg_.prior_rot_sigma_rad,
                                   cfg_.prior_trans_sigma_m, cfg_.prior_trans_sigma_m, cfg_.prior_trans_sigma_m)
                                      .finished();
    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(xk, toGtsamPose3_(T_OB_init), priorNoise));
    prior_added++;

    // Velocity prior
    auto velNoise = gtsam::noiseModel::Isotropic::Sigma(3, cfg_.vel_prior_sigma);
    newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(vk, gtsam::Vector3(0.0, 0.0, 0.0), velNoise));
    prior_added++;

    // Bias prior
    const gtsam::Vector6 bsig = (gtsam::Vector6() << cfg_.bias_acc_prior_sigma, cfg_.bias_acc_prior_sigma, cfg_.bias_acc_prior_sigma,
                                 cfg_.bias_gyro_prior_sigma, cfg_.bias_gyro_prior_sigma, cfg_.bias_gyro_prior_sigma)
                                    .finished();
    auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas(bsig);
    newFactors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bk, gtsam::imuBias::ConstantBias(), biasNoise));
    prior_added++;
  }

  // Optional VO BetweenFactor on BODY poses
  int between_added = 0;
  bool had_vo_between_measurement = initialized_ && T_Bkm1_Bk_meas.has_value();
  bool skipped_between = false;
  double interval_quality = 1.0;
  double sigma_scale = 1.0;
  if (cfg_.use_vo_between && initialized_ && T_Bkm1_Bk_meas.has_value())
  {
    const gtsam::Key xkm1 = X_(last_kf_id_);
    const gtsam::Pose3 meas = toGtsamPose3_(*T_Bkm1_Bk_meas);
    if (cfg_.use_interval_health_for_vo_between)
    {
      interval_quality = computeVoBetweenIntervalQuality_(kf.interval_health, cfg_);
      sigma_scale = 1.0 + (1.0 - interval_quality) * std::max(0.0, cfg_.between_health_max_sigma_scale - 1.0);
      skipped_between = shouldSkipVoBetweenInterval_(kf.interval_health, cfg_, interval_quality);
    }

    if (!skipped_between)
    {
      const gtsam::Vector6 sigmas =
          (gtsam::Vector6() << cfg_.between_rot_sigma_rad * sigma_scale, cfg_.between_rot_sigma_rad * sigma_scale, cfg_.between_rot_sigma_rad * sigma_scale,
           cfg_.between_trans_sigma_m * sigma_scale, cfg_.between_trans_sigma_m * sigma_scale, cfg_.between_trans_sigma_m * sigma_scale)
              .finished();
      auto betweenNoise = maybeHuberize_(
          gtsam::noiseModel::Diagonal::Sigmas(sigmas),
          cfg_.between_huber_k);

      newFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(xkm1, xk, meas, betweenNoise));
      between_added = 1;
    }

    if (cfg_.use_interval_health_for_vo_between && (skipped_between || sigma_scale > 1.01))
    {
      std::cout << "[Optimizer] kf_id=" << kf.kf_id
                << " vo_between_quality=" << interval_quality
                << " sigma_scale=" << sigma_scale
                << " skip_between=" << (skipped_between ? 1 : 0)
                << " frames=" << kf.interval_health.num_frames
                << " pose_valid=" << kf.interval_health.num_pose_valid_frames
                << " degraded=" << kf.interval_health.num_degraded_frames
                << " lost=" << kf.interval_health.num_lost_frames
                << "\n";
    }
  }

  // IMU factor between last and current keyframe (after first)
  int imu_factors_added = 0;
  if (cfg_.use_imu && initialized_)
  {
    gtsam::PreintegratedCombinedMeasurements pim;
    if (!deserializePim_(kf.pim_bytes, &pim))
    {
      std::cout << "[Optimizer] IMU deserialize failed on kf_id=" << kf.kf_id << "\n";
      return std::nullopt;
    }

    const uint64_t km1 = last_kf_id_;
    newFactors.add(gtsam::CombinedImuFactor(
        X_(km1), V_(km1),
        xk, V_(kf.kf_id),
        B_(km1), B_(kf.kf_id),
        pim));
    imu_factors_added = 1;
  }

  double stereo_interval_quality = 1.0;
  double stereo_sigma_scale = 1.0;
  bool skipped_stereo = false;
  if (cfg_.use_interval_health_for_vo_between)
  {
    stereo_interval_quality = computeVoBetweenIntervalQuality_(kf.interval_health, cfg_);
    stereo_sigma_scale =
        1.0 + (1.0 - stereo_interval_quality) * std::max(0.0, cfg_.between_health_max_sigma_scale - 1.0);
    skipped_stereo = shouldSkipVoBetweenInterval_(kf.interval_health, cfg_, stereo_interval_quality);
  }

  auto stereoNoise = maybeHuberize_(
      gtsam::noiseModel::Isotropic::Sigma(3, cfg_.stereo_sigma_px * stereo_sigma_scale),
      cfg_.stereo_huber_k);
  using StereoFactor = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;

  const auto &Lcam = cfg_.rig.left;
  const double fx = static_cast<double>(Lcam.fx());
  const double fy = static_cast<double>(Lcam.fy());
  const double cx = static_cast<double>(Lcam.cx());
  const double cy = static_cast<double>(Lcam.cy());
  const double b = cfg_.rig.baseline;

  int stereo_factors_added = 0;
  int landmarks_created = 0;

  // Camera pose derived from body init (consistency)
  const Eigen::Isometry3d T_WC_from_body = T_OB_init * cfg_.T_BC;

  if (!skipped_stereo)
  {
    for (size_t i = 0; i < N; ++i)
    {
      if (kf.has_r[i] == 0)
        continue;

      const TrackId tid = kf.ids[i];
      const gtsam::Key lk = L_(tid);

      const bool new_landmark = (landmark_last_seen_kf_.find(tid) == landmark_last_seen_kf_.end());

      const double uL = static_cast<double>(kf.pl[i].x);
      const double v = static_cast<double>(kf.pl[i].y);
      const double uR = static_cast<double>(kf.pr[i].x);

      const gtsam::StereoPoint2 z(uL, uR, v);

      newFactors.add(StereoFactor(z, stereoNoise, xk, lk, Kst_, body_T_cam_));
      ++stereo_factors_added;

      if (new_landmark && cfg_.init_landmarks_from_stereo)
      {
        const double disp = uL - uR;
        if (disp > 1e-9)
        {
          const Eigen::Vector3d pC = triangulateStereoInLeftCam_(uL, uR, v, fx, fy, cx, cy, b);
          const Eigen::Vector3d pW = T_WC_from_body * pC;
          newValues.insert(lk, toGtsamPoint3_(pW));
          ++landmarks_created;
        }
      }

      timestamps[lk] = t_now;
      landmark_last_seen_kf_[tid] = kf.kf_id;
    }
  }

  if (cfg_.use_interval_health_for_vo_between && (skipped_stereo || stereo_sigma_scale > 1.01))
  {
    std::cout << "[Optimizer] kf_id=" << kf.kf_id
              << " stereo_quality=" << stereo_interval_quality
              << " stereo_sigma_scale=" << stereo_sigma_scale
              << " skip_stereo=" << (skipped_stereo ? 1 : 0)
              << " frames=" << kf.interval_health.num_frames
              << " pose_valid=" << kf.interval_health.num_pose_valid_frames
              << " degraded=" << kf.interval_health.num_degraded_frames
              << " lost=" << kf.interval_health.num_lost_frames
              << "\n";
  }

  gtsam::FixedLagSmoother::Result update_result;
  try
  {
    update_result = smoother_.update(newFactors, newValues, timestamps);
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] smoother_.update exception: " << e.what() << std::endl;
    return std::nullopt;
  }
  catch (...)
  {
    std::cout << "[Optimizer] smoother_.update unknown exception\n";
    return std::nullopt;
  }

  OptimizationResult out;
  out.kf_id = kf.kf_id;
  out.t_s = kf.t_end;

  try
  {
    const gtsam::Pose3 T_WB_opt_g = smoother_.calculateEstimate<gtsam::Pose3>(xk);
    out.T_WB_opt = toEigenIsometry3d_(T_WB_opt_g);
    out.T_WC_opt = out.T_WB_opt * cfg_.T_BC;
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] calculateEstimate exception: " << e.what() << std::endl;
    return std::nullopt;
  }
  catch (...)
  {
    std::cout << "[Optimizer] calculateEstimate unknown exception\n";
    return std::nullopt;
  }

  try
  {
    const gtsam::Vector3 v_k = smoother_.calculateEstimate<gtsam::Vector3>(vk);
    out.velocity_opt = Eigen::Vector3d(v_k.x(), v_k.y(), v_k.z());
    out.has_velocity = true;
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] Failed to read velocity estimate: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "[Optimizer] Failed to read velocity estimate (unknown)\n";
  }

  try
  {
    const gtsam::imuBias::ConstantBias b_k =
        smoother_.calculateEstimate<gtsam::imuBias::ConstantBias>(B_(kf.kf_id));

    // Fill your OptimizationResult (assuming you add these fields)
    out.bias_opt.accel = Eigen::Vector3d(b_k.accelerometer().x(),
                                         b_k.accelerometer().y(),
                                         b_k.accelerometer().z());
    out.bias_opt.gyro = Eigen::Vector3d(b_k.gyroscope().x(),
                                        b_k.gyroscope().y(),
                                        b_k.gyroscope().z());
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] Failed to read bias estimate: " << e.what() << std::endl;
    // leave bias as default / zero
  }
  catch (...)
  {
    std::cout << "[Optimizer] Failed to read bias estimate (unknown)\n";
  }

  try
  {
    out.pose_wb_covariance = toRowMajorArray_<6, 6>(smoother_.marginalCovariance(xk));
    out.has_pose_wb_covariance = true;
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] Failed to read pose covariance: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "[Optimizer] Failed to read pose covariance (unknown)\n";
  }

  try
  {
    out.velocity_covariance = toRowMajorArray_<3, 3>(smoother_.marginalCovariance(vk));
    out.has_velocity_covariance = true;
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] Failed to read velocity covariance: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "[Optimizer] Failed to read velocity covariance (unknown)\n";
  }

  try
  {
    out.bias_covariance = toRowMajorArray_<6, 6>(smoother_.marginalCovariance(bk));
    out.has_bias_covariance = true;
  }
  catch (const std::exception &e)
  {
    std::cout << "[Optimizer] Failed to read bias covariance: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "[Optimizer] Failed to read bias covariance (unknown)\n";
  }

  const auto &isam_result = smoother_.getISAM2Result();
  out.stats.num_keyframes_in_window = static_cast<int>(cfg_.window_size);
  out.stats.num_landmarks_alive = static_cast<int>(landmark_last_seen_kf_.size());
  out.stats.num_landmarks_created = landmarks_created;
  out.stats.num_stereo_factors_added = stereo_factors_added;
  out.stats.num_imu_factors_added = imu_factors_added;
  out.stats.num_between_factors_added = between_added;
  out.stats.num_prior_factors_added = prior_added;
  out.stats.had_vo_between_measurement = had_vo_between_measurement;
  out.stats.used_vo_between_factor = between_added > 0;
  out.stats.skipped_vo_between_factor = had_vo_between_measurement && skipped_between;
  out.stats.vo_between_quality = interval_quality;
  out.stats.vo_between_sigma_scale = sigma_scale;
  out.stats.imu_only_update =
      (imu_factors_added > 0) &&
      (stereo_factors_added == 0) &&
      (between_added == 0);
  out.stats.update_iterations = static_cast<int>(update_result.iterations);
  out.stats.update_intermediate_steps = static_cast<int>(update_result.intermediateSteps);
  out.stats.update_nonlinear_variables = static_cast<int>(update_result.nonlinearVariables);
  out.stats.update_linear_variables = static_cast<int>(update_result.linearVariables);
  out.stats.final_error = update_result.error;
  out.stats.has_error_before = isam_result.errorBefore.has_value();
  out.stats.error_before = isam_result.errorBefore.value_or(-1.0);
  out.stats.has_error_after = isam_result.errorAfter.has_value();
  out.stats.error_after = isam_result.errorAfter.value_or(-1.0);
  out.stats.variables_relinearized = static_cast<int>(isam_result.variablesRelinearized);
  out.stats.variables_reeliminated = static_cast<int>(isam_result.variablesReeliminated);
  out.stats.factors_recalculated = static_cast<int>(isam_result.factorsRecalculated);
  out.stats.cliques = static_cast<int>(isam_result.cliques);

  initialized_ = true;
  last_kf_id_ = kf.kf_id;

  return out;
}
