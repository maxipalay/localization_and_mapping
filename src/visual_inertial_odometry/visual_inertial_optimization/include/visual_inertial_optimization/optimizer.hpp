#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include "visual_inertial_optimization/optimization.hpp"

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
        reset();
        rebuildCalibration_();
        rebuildExtrinsics_();
    }

    void reset()
    {
        initialized_ = false;
        last_kf_id_ = 0;

        const double lag_kf = static_cast<double>(cfg_.window_size > 0 ? (cfg_.window_size - 1) : 0);
        smoother_ = gtsam::IncrementalFixedLagSmoother(lag_kf);

        landmark_last_seen_kf_.clear();
        active_kf_ids_.clear();
    }

    std::optional<OptimizationResult> push(
        const KeyframeEvent &kf,
        const std::optional<Eigen::Isometry3d> &T_Bkm1_Bk_meas = std::nullopt,
        const std::vector<AbsolutePosePrior> &absolute_pose_priors = {},
        const std::optional<Eigen::Isometry3d> &T_WB_init_override = std::nullopt,
        const std::optional<Eigen::Isometry3d> &T_WB_anchor_override = std::nullopt);

    std::vector<LandmarkEstimate> getLandmarks(size_t max_points = 0) const;

private:
    static inline gtsam::Key X_(uint64_t kf_id) { return gtsam::Symbol('x', kf_id); }
    static inline gtsam::Key V_(uint64_t kf_id) { return gtsam::Symbol('v', kf_id); }
    static inline gtsam::Key B_(uint64_t kf_id) { return gtsam::Symbol('b', kf_id); }
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
        body_T_cam_ = toGtsamPose3_(cfg_.T_BC);
    }

private:
    OptimizationConfig cfg_;

    gtsam::IncrementalFixedLagSmoother smoother_;
    gtsam::Cal3_S2Stereo::shared_ptr Kst_;
    std::optional<gtsam::Pose3> body_T_cam_;
    bool initialized_ = false;
    uint64_t last_kf_id_ = 0;
    std::deque<uint64_t> active_kf_ids_;

    std::unordered_map<TrackId, uint64_t> landmark_last_seen_kf_;

    mutable std::mutex mtx_;
};
