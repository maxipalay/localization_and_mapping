#pragma once

#include <cstdint>

#include <Eigen/Geometry>

#include <visual_inertial_localization/localization_types.hpp>

namespace visual_inertial_localization
{

class LocalizationEngine;

class LocalizationController
{
public:
    explicit LocalizationController(LocalizationEngine &engine);

    LocalizationDecision processKeyframe(
        int64_t stamp_ns,
        const Eigen::Isometry3d &T_OB) const;

    void updateMapOdomEstimate(const Eigen::Isometry3d &T_MO) const noexcept;
    LocalizationState state() const noexcept;

private:
    LocalizationPosePrior makePrior_(
        const Eigen::Isometry3d &T_WB,
        bool robust) const;
    void appendPosePriorEstimates_(
        LocalizationDecision &decision,
        int64_t stamp_ns) const;

    LocalizationEngine &engine_;
    mutable LocalizationState state_{LocalizationState::OdomOnly};
    mutable Eigen::Isometry3d T_MO_ = Eigen::Isometry3d::Identity();
};

} // namespace visual_inertial_localization
