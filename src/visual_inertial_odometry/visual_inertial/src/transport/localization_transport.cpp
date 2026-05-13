#include "visual_inertial/transport/localization_transport.hpp"

#include "visual_inertial/transport/common_transport.hpp"

namespace visual_inertial::transport
{

visual_inertial::msg::LocalizationPosePrior makeLocalizationPosePriorMsg(
    const visual_inertial_localization::LocalizationPosePrior &prior)
{
    visual_inertial::msg::LocalizationPosePrior msg;
    msg.pose_wb = isoToPoseMsg(prior.T_WB);
    msg.rot_sigma_rad = prior.rot_sigma_rad;
    msg.trans_sigma_m = prior.trans_sigma_m;
    msg.huber_k = prior.huber_k;
    return msg;
}

visual_inertial::msg::LocalizationCommand makeLocalizationCommandMsg(
    const visual_inertial::msg::Keyframe &keyframe,
    const visual_inertial_localization::LocalizationDecision &decision)
{
    visual_inertial::msg::LocalizationCommand msg;
    msg.header = keyframe.header;
    msg.kf_id = keyframe.kf_id;
    msg.state =
        (decision.state == visual_inertial_localization::LocalizationState::Localized)
            ? visual_inertial::msg::LocalizationCommand::STATE_LOCALIZED
            : visual_inertial::msg::LocalizationCommand::STATE_ODOM_ONLY;

    switch (decision.action)
    {
    case visual_inertial_localization::LocalizationAction::Bootstrap:
        msg.action = visual_inertial::msg::LocalizationCommand::ACTION_BOOTSTRAP;
        break;
    case visual_inertial_localization::LocalizationAction::Relocalize:
        msg.action = visual_inertial::msg::LocalizationCommand::ACTION_RELOCALIZE;
        break;
    case visual_inertial_localization::LocalizationAction::None:
    default:
        msg.action = visual_inertial::msg::LocalizationCommand::ACTION_NONE;
        break;
    }

    msg.waiting_for_bootstrap = decision.waiting_for_bootstrap;
    msg.absolute_pose_priors.reserve(decision.optimizer_inputs.absolute_pose_priors.size());
    for (const auto &prior : decision.optimizer_inputs.absolute_pose_priors)
    {
        msg.absolute_pose_priors.push_back(makeLocalizationPosePriorMsg(prior));
    }

    msg.has_init_override = decision.optimizer_inputs.T_WB_init_override.has_value();
    if (msg.has_init_override)
    {
        msg.pose_wb_init_override = isoToPoseMsg(*decision.optimizer_inputs.T_WB_init_override);
    }

    msg.has_anchor_override = decision.optimizer_inputs.T_WB_anchor_override.has_value();
    if (msg.has_anchor_override)
    {
        msg.pose_wb_anchor_override = isoToPoseMsg(*decision.optimizer_inputs.T_WB_anchor_override);
    }

    msg.has_bootstrap_info = decision.bootstrap.has_value();
    if (msg.has_bootstrap_info)
    {
        msg.bootstrap_support_count = static_cast<uint32_t>(decision.bootstrap->support_count);
        msg.bootstrap_score = decision.bootstrap->score;
    }

    msg.has_relocalization_info = decision.relocalization.has_value();
    if (msg.has_relocalization_info)
    {
        msg.relocalization_frame_support =
            static_cast<uint32_t>(decision.relocalization->frame_support);
        msg.relocalization_support_count =
            static_cast<uint32_t>(decision.relocalization->support_count);
        msg.relocalization_translation_error_m =
            decision.relocalization->translation_error_m;
        msg.relocalization_rotation_error_rad =
            decision.relocalization->rotation_error_rad;
    }

    return msg;
}

AbsolutePosePrior toAbsolutePosePrior(
    const visual_inertial::msg::LocalizationPosePrior &prior)
{
    AbsolutePosePrior out;
    out.T_WB = poseMsgToIso(prior.pose_wb);
    out.rot_sigma_rad = prior.rot_sigma_rad;
    out.trans_sigma_m = prior.trans_sigma_m;
    out.huber_k = prior.huber_k;
    return out;
}

} // namespace visual_inertial::transport
