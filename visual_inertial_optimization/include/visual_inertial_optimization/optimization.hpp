#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>

#include <Eigen/Geometry>

#include "visual_inertial_common/types.hpp"
#include "visual_inertial_optimization/types.hpp"

#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/StereoFactor.h>

// This backend maintains an incremental fixed-lag smoother using:
//  - Pose variables X(kf_id)
//  - Landmark variables L(track_id)
//  - Factors:
//      * gtsam::GenericStereoFactor<Pose3, Point3>  (stereo measurement)
//      * gtsam::PriorFactor<Pose3> (gauge fix / anchor)
//      * optional gtsam::BetweenFactor<Pose3> from frontend VO (helps stability early on)
//
// Notes:
// - Requires rectified stereo for StereoPoint2(uL,uR,v). Distortion should be handled upstream.
// - Landmarks are keyed by track_id and live as long as they stay inside the fixed-lag horizon.

struct OptimizationConfig
{
    // Keep ~window_size keyframes alive. Implemented as lag_kf = window_size - 1.
    size_t window_size = 8;

    // Rectified stereo calibration. Must satisfy rig.valid().
    CameraRig rig;

    // Noise model for stereo reprojection factors (pixels)
    double stereo_sigma_px = 1.0;

    // Gauge prior on first pose ever inserted (or after reset)
    double prior_rot_sigma_rad = 5.0 * M_PI / 180.0;
    double prior_trans_sigma_m = 0.25;

    // Optional VO between factor between consecutive keyframes
    bool use_vo_between = true;
    double between_rot_sigma_rad = 3.0 * M_PI / 180.0;
    double between_trans_sigma_m = 0.10;

    // Landmark initialization from stereo in the current keyframe
    bool init_landmarks_from_stereo = true;

    // Bookkeeping: remove landmarks from internal maps when they are no longer in the smoother.
    bool prune_unobserved_landmarks = true;

    // Fixed extrinsic: T_BC (Body <- CameraOptical)
    // Rotation: optical (x right, y down, z forward) -> body/ROS (x forward, y left, z up)
    // Translation: camera is +0.07m in body +Y (left)
    static Eigen::Isometry3d default_T_BC()
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() << 0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0;
        T.translation() = Eigen::Vector3d(0.0, 0.072, 0.0);
        return T;
    }

    Eigen::Isometry3d T_BC = default_T_BC();
};

class Optimizer
{
public:
    using TrackId = KeyframeEvent::TrackId;

    explicit Optimizer(const OptimizationConfig &cfg)
        : cfg_(cfg),
          smoother_(static_cast<double>(cfg.window_size > 0 ? (cfg.window_size - 1) : 0))
    {
        rebuildCalibration_();
        rebuildExtrinsics_();
    }

    const OptimizationConfig &config() const { return cfg_; }

    void setConfig(const OptimizationConfig &cfg)
    {
        cfg_ = cfg;
        const double lag_kf = static_cast<double>(cfg_.window_size > 0 ? (cfg_.window_size - 1) : 0);
        smoother_ = gtsam::IncrementalFixedLagSmoother(lag_kf);
        reset();               // simplest + safest for now
        rebuildCalibration_(); // rebuild Kst_
        rebuildExtrinsics_();
    }

    void reset()
    {
        initialized_ = false;
        last_kf_id_ = 0;

        const double lag_kf = static_cast<double>(cfg_.window_size > 0 ? (cfg_.window_size - 1) : 0);
        smoother_ = gtsam::IncrementalFixedLagSmoother(lag_kf);

        landmark_last_seen_kf_.clear();
    }

    // Push one keyframe; runs smoother.update() with:
    //  - new stereo factors for each observation in this keyframe (where has_r[i] != 0)
    //  - new pose initial value
    //  - new landmark initial values for tracks first seen (if init_landmarks_from_stereo)
    //  - timestamps where time := double(kf_id)
    //
    // Optional relative motion input:
    //   T_Ck_Ckm1 is (Ck <- Ck-1) from your PnP convention.
    // If cfg.use_vo_between==true and T_Ck_Ckm1 is provided, adds a BetweenFactor between X(k-1) and X(k).
    std::optional<OptimizationResult> push(
        const KeyframeEvent &kf,
        const std::optional<Eigen::Isometry3d> &T_Ck_Ckm1 = std::nullopt);

private:
    static inline gtsam::Key X_(uint64_t kf_id) { return gtsam::Symbol('x', kf_id); }
    static inline gtsam::Key L_(TrackId tid) { return gtsam::Symbol('l', static_cast<uint64_t>(tid)); }

    static inline gtsam::Pose3 toGtsamPose3_(const Eigen::Isometry3d &T)
    {
        const Eigen::Matrix3d R = T.linear();
        const Eigen::Vector3d t = T.translation();
        const Eigen::Quaterniond q(R);
        return gtsam::Pose3(
            gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
            gtsam::Point3(t.x(), t.y(), t.z()));
    }

    static inline Eigen::Isometry3d toEigenIsometry3d_(const gtsam::Pose3 &T)
    {
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        const gtsam::Matrix3 R = T.rotation().matrix();
        const gtsam::Point3 t = T.translation();
        out.linear() << R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2);
        out.translation() = Eigen::Vector3d(t.x(), t.y(), t.z());
        return out;
    }

    void rebuildCalibration_()
    {
        // Use LEFT intrinsics + baseline; assumes rectified stereo model.
        const auto &L = cfg_.rig.left;
        Kst_ = std::make_shared<gtsam::Cal3_S2Stereo>(
            static_cast<double>(L.fx()),
            static_cast<double>(L.fy()),
            0.0,
            static_cast<double>(L.cx()),
            static_cast<double>(L.cy()),
            cfg_.rig.baseline);
    }

    void rebuildExtrinsics_()
    {
        body_T_cam_ = toGtsamPose3_(cfg_.T_BC); // (Body <- CameraOptical)
    }

private:
    OptimizationConfig cfg_;

    gtsam::IncrementalFixedLagSmoother smoother_;
    gtsam::Cal3_S2Stereo::shared_ptr Kst_;
    std::optional<gtsam::Pose3> body_T_cam_; // passed into stereo factors
    bool initialized_ = false;
    uint64_t last_kf_id_ = 0;

    // Track/landmark bookkeeping: last keyframe id where landmark was observed.
    // Used for optional pruning and for refreshing timestamps for fixed-lag.
    std::unordered_map<TrackId, uint64_t> landmark_last_seen_kf_;
};