#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include "visual_inertial_common/types.hpp"
#include "visual_inertial_optimization/types.hpp"

struct LandmarkEstimate
{
    using TrackId = KeyframeEvent::TrackId;

    TrackId tid = 0;
    Eigen::Vector3d p_W = Eigen::Vector3d::Zero();
    uint64_t last_seen_kf = 0;
};

struct OptimizationConfig
{
    size_t window_size = 8;

    CameraRig rig;

    double stereo_sigma_px = 1.0;
    double stereo_huber_k = 1.345;

    double prior_rot_sigma_rad = 5.0 * M_PI / 180.0;
    double prior_trans_sigma_m = 0.25;

    double vel_prior_sigma = 1.0;
    double bias_acc_prior_sigma = 0.5;
    double bias_gyro_prior_sigma = 0.1;

    bool use_vo_between = true;
    double between_rot_sigma_rad = 3.0 * M_PI / 180.0;
    double between_trans_sigma_m = 0.10;
    double between_huber_k = 1.345;
    bool use_interval_health_for_vo_between = true;
    double between_health_min_pose_valid_fraction = 0.6;
    double between_health_min_track_retention = 0.6;
    double between_health_min_pnp_inlier_ratio = 0.25;
    double between_health_min_track_coverage = 0.2;
    double between_health_max_pnp_reproj_rmse_px = 3.0;
    double between_health_max_sigma_scale = 5.0;
    double between_health_skip_quality = 0.05;

    bool use_imu = true;

    bool init_landmarks_from_stereo = true;
    bool prune_unobserved_landmarks = true;

    static Eigen::Isometry3d default_T_BC()
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() << 0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0;
        T.translation() = Eigen::Vector3d(0.0, 0.05, 0.0);
        return T;
    }

    Eigen::Isometry3d T_BC = default_T_BC();
};

class Optimizer;

class OptimizationModule
{
public:
    explicit OptimizationModule(const OptimizationConfig &config)
        : config_(config)
    {
    }

    const OptimizationConfig &config() const noexcept
    {
        return config_;
    }

    bool ready() const noexcept;

    void setBodyCameraExtrinsic(const Eigen::Isometry3d &T_BC);
    bool initializeRig(const CameraRig &rig);

    void reset();

    std::optional<OptimizationResult> push(
        const KeyframeEvent &kf,
        const std::optional<Eigen::Isometry3d> &T_Bkm1_Bk_meas = std::nullopt,
        const std::vector<AbsolutePosePrior> &absolute_pose_priors = {},
        const std::optional<Eigen::Isometry3d> &T_WB_init_override = std::nullopt,
        const std::optional<Eigen::Isometry3d> &T_WB_anchor_override = std::nullopt);

    std::vector<LandmarkEstimate> getLandmarks(size_t max_points = 0) const;

private:
    OptimizationConfig config_;
    std::shared_ptr<Optimizer> optimizer_;
    mutable std::mutex mtx_;
};
