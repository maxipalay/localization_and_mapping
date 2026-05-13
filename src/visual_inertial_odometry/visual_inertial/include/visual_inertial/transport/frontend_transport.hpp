#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <visual_inertial/msg/frontend_health.hpp>
#include <visual_inertial/msg/keyframe.hpp>
#include <visual_inertial/msg/tracks.hpp>
#include <visual_inertial_common/types.hpp>
#include <visual_inertial_frontend/types.hpp>

#include <opencv2/core/types.hpp>

#include <string>
#include <vector>

namespace visual_inertial::transport
{

visual_inertial::msg::Tracks makeTracksMsg(
    const builtin_interfaces::msg::Time &stamp,
    const std::vector<cv::Point2f> &tracks);

visual_inertial::msg::FrontendHealth makeFrontendHealthMsg(
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    const FrontendHealth &health);

visual_inertial::msg::Keyframe makeKeyframeMsg(
    const KeyframeEvent &event,
    const std::string &frame_id);

} // namespace visual_inertial::transport
