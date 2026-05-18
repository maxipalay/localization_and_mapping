#pragma once

#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/localization_command.hpp>
#include <visual_inertial/msg/localization_pose_prior.hpp>
#include <visual_inertial_localization/localization_types.hpp>
#include <visual_inertial_optimization/types.hpp>

namespace visual_inertial::transport
{

visual_inertial::msg::LocalizationPosePrior makeLocalizationPosePriorMsg(
    const visual_inertial_localization::LocalizationPosePrior &prior);

visual_inertial::msg::LocalizationCommand makeLocalizationCommandMsg(
    const visual_inertial::msg::Keyframe &keyframe,
    const visual_inertial_localization::LocalizationDecision &decision);

AbsolutePosePrior toAbsolutePosePrior(
    const visual_inertial::msg::LocalizationPosePrior &prior);

} // namespace visual_inertial::transport
