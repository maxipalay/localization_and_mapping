#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace visual_inertial_localization
{

enum class LocalizationState
{
    OdomOnly,
    Localized,
};

enum class LocalizationAction
{
    None,
    Bootstrap,
    Relocalize,
};

struct LocalizationPosePrior
{
    Eigen::Isometry3d T_WB = Eigen::Isometry3d::Identity();
    double rot_sigma_rad{0.35};
    double trans_sigma_m{0.10};
    double huber_k{1.0};
};

struct LocalizationOptimizerInputs
{
    std::vector<LocalizationPosePrior> absolute_pose_priors;
    std::optional<Eigen::Isometry3d> T_WB_init_override;
    std::optional<Eigen::Isometry3d> T_WB_anchor_override;
};

struct LocalizationBootstrapInfo
{
    size_t support_count{0};
    double score{0.0};
};

struct LocalizationRelocalizationInfo
{
    size_t frame_support{0};
    size_t support_count{0};
    double translation_error_m{0.0};
    double rotation_error_rad{0.0};
};

struct LocalizationDecision
{
    LocalizationState state{LocalizationState::OdomOnly};
    LocalizationAction action{LocalizationAction::None};
    LocalizationOptimizerInputs optimizer_inputs;
    bool waiting_for_bootstrap{false};
    std::optional<LocalizationBootstrapInfo> bootstrap;
    std::optional<LocalizationRelocalizationInfo> relocalization;
};

} // namespace visual_inertial_localization
