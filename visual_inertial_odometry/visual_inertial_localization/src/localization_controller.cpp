#include <visual_inertial_localization/localization_controller.hpp>
#include <visual_inertial_localization/localization_engine.hpp>

#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>

namespace visual_inertial_localization
{

namespace
{

double poseTranslationDistance(
    const Eigen::Isometry3d &T_a,
    const Eigen::Isometry3d &T_b)
{
    return (T_a.translation() - T_b.translation()).norm();
}

double poseRotationDistanceRad(
    const Eigen::Isometry3d &T_a,
    const Eigen::Isometry3d &T_b)
{
    Eigen::Quaterniond q_rel(T_a.linear().transpose() * T_b.linear());
    q_rel.normalize();
    const double w = std::clamp(std::abs(q_rel.w()), 0.0, 1.0);
    return 2.0 * std::acos(w);
}

} // namespace

LocalizationController::LocalizationController(LocalizationEngine &engine)
    : engine_(engine)
{
}

LocalizationDecision LocalizationController::processKeyframe(
    int64_t stamp_ns,
    const Eigen::Isometry3d &T_OB) const
{
    LocalizationDecision decision;
    decision.state = state_;

    const auto stamp = rclcpp::Time(stamp_ns);

    if (state_ == LocalizationState::OdomOnly)
    {
        const auto bootstrap = engine_.estimateBootstrap(stamp, T_OB);
        if (!bootstrap.has_value())
        {
            decision.waiting_for_bootstrap = true;
            return decision;
        }

        T_MO_ = bootstrap->T_MO;
        state_ = LocalizationState::Localized;

        decision.action = LocalizationAction::Bootstrap;
        decision.bootstrap = LocalizationBootstrapInfo{
            bootstrap->support_count,
            bootstrap->score};
        decision.optimizer_inputs.T_WB_init_override = T_MO_ * T_OB;
        decision.optimizer_inputs.T_WB_anchor_override =
            decision.optimizer_inputs.T_WB_init_override;

        appendPosePriorEstimates_(decision, stamp_ns);
        if (decision.optimizer_inputs.absolute_pose_priors.empty())
        {
            decision.optimizer_inputs.absolute_pose_priors.push_back(
                makePrior_(*decision.optimizer_inputs.T_WB_anchor_override, false));
        }
    }
    else
    {
        decision.optimizer_inputs.T_WB_init_override = T_MO_ * T_OB;
    }

    if (state_ == LocalizationState::Localized)
    {
        const auto stable_correction = engine_.estimateStableCorrection(stamp);
        const auto relocalization_correction = engine_.estimateRelocalizationCorrection(stamp);
        bool used_stable_correction = false;

        if (relocalization_correction.has_value())
        {
            const double relocalize_rotation_thresh_rad =
                engine_.config().relocalize_rotation_deg * std::acos(-1.0) / 180.0;
            const double translation_error_m =
                poseTranslationDistance(T_MO_, relocalization_correction->T_MO);
            const double rotation_error_rad =
                poseRotationDistanceRad(T_MO_, relocalization_correction->T_MO);

            if (translation_error_m > engine_.config().relocalize_translation_m ||
                rotation_error_rad > relocalize_rotation_thresh_rad)
            {
                T_MO_ = relocalization_correction->T_MO;
                decision.action = LocalizationAction::Relocalize;
                decision.relocalization = LocalizationRelocalizationInfo{
                    relocalization_correction->frame_support,
                    relocalization_correction->support_count,
                    translation_error_m,
                    rotation_error_rad};
                decision.optimizer_inputs.T_WB_init_override = T_MO_ * T_OB;
                decision.optimizer_inputs.T_WB_anchor_override =
                    decision.optimizer_inputs.T_WB_init_override;
                decision.optimizer_inputs.absolute_pose_priors.clear();
                decision.optimizer_inputs.absolute_pose_priors.push_back(
                    makePrior_(*decision.optimizer_inputs.T_WB_anchor_override, false));
                // A hard relocalization should not keep reusing the evidence that triggered it.
                engine_.clearTemporalState();
                used_stable_correction = true;
            }
        }

        if (!used_stable_correction && stable_correction.has_value())
        {
            const double tracking_deadband_rotation_thresh_rad =
                engine_.config().tracking_deadband_rotation_deg * std::acos(-1.0) / 180.0;
            const double translation_error_m =
                poseTranslationDistance(T_MO_, stable_correction->T_MO);
            const double rotation_error_rad =
                poseRotationDistanceRad(T_MO_, stable_correction->T_MO);

            decision.optimizer_inputs.absolute_pose_priors.clear();
            if (translation_error_m > engine_.config().tracking_deadband_translation_m ||
                rotation_error_rad > tracking_deadband_rotation_thresh_rad)
            {
                decision.optimizer_inputs.absolute_pose_priors.push_back(
                    makePrior_(stable_correction->T_MO * T_OB, true));
            }

            used_stable_correction = true;
        }

        if (!used_stable_correction)
        {
            decision.optimizer_inputs.absolute_pose_priors.clear();
            appendPosePriorEstimates_(decision, stamp_ns);
        }
    }

    decision.state = state_;
    return decision;
}

void LocalizationController::updateMapOdomEstimate(const Eigen::Isometry3d &T_MO) const noexcept
{
    if (state_ == LocalizationState::Localized)
    {
        T_MO_ = T_MO;
    }
}

LocalizationState LocalizationController::state() const noexcept
{
    return state_;
}

LocalizationPosePrior LocalizationController::makePrior_(
    const Eigen::Isometry3d &T_WB,
    bool robust) const
{
    LocalizationPosePrior prior;
    prior.T_WB = T_WB;
    prior.rot_sigma_rad = engine_.config().pose_prior_rot_sigma_rad;
    prior.trans_sigma_m = engine_.config().pose_prior_trans_sigma_m;
    prior.huber_k = robust ? engine_.config().pose_prior_huber_k : 0.0;
    return prior;
}

void LocalizationController::appendPosePriorEstimates_(
    LocalizationDecision &decision,
    int64_t stamp_ns) const
{
    const auto pose_prior_estimates = engine_.estimatePosePriors(rclcpp::Time(stamp_ns));
    for (const auto &estimate : pose_prior_estimates)
    {
        decision.optimizer_inputs.absolute_pose_priors.push_back(
            makePrior_(estimate.T_MB, true));
    }
}

} // namespace visual_inertial_localization
