#include "visual_inertial/transport/frontend_transport.hpp"

#include "visual_inertial/transport/common_transport.hpp"

namespace visual_inertial::transport
{

visual_inertial::msg::Tracks makeTracksMsg(
    const builtin_interfaces::msg::Time &stamp,
    const std::vector<cv::Point2f> &tracks)
{
    visual_inertial::msg::Tracks msg;
    msg.header.stamp = stamp;

    const auto count = tracks.size();
    msg.u_l.resize(count);
    msg.v_l.resize(count);

    for (size_t i = 0; i < count; ++i)
    {
        msg.u_l[i] = tracks[i].x;
        msg.v_l[i] = tracks[i].y;
    }

    return msg;
}

visual_inertial::msg::FrontendHealth makeFrontendHealthMsg(
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    const FrontendHealth &health)
{
    visual_inertial::msg::FrontendHealth msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.num_tracks = health.num_tracks;
    msg.num_stereo_tracks = health.num_stereo_tracks;
    msg.num_pnp_candidates = health.num_pnp_candidates;
    msg.num_pnp_inliers = health.num_pnp_inliers;
    msg.pnp_inlier_ratio = health.pnp_inlier_ratio;
    msg.pnp_reproj_rmse_px = health.pnp_reproj_rmse_px;
    msg.track_coverage = health.track_coverage;
    msg.track_retention = health.track_retention;
    msg.pose_update_valid = health.pose_update_valid;
    msg.state = health.state;
    return msg;
}

visual_inertial::msg::Keyframe makeKeyframeMsg(
    const KeyframeEvent &event,
    const std::string &frame_id)
{
    visual_inertial::msg::Keyframe msg;
    msg.header.stamp = toBuiltinTime(event.t_end);
    msg.header.frame_id = frame_id;
    msg.kf_id = event.kf_id;
    msg.t_start = event.t_start;
    msg.t_end = event.t_end;
    msg.pose_odom_body = isoToPoseMsg(event.T_OB);

    msg.has_vo_between = event.has_vo_between ? uint8_t{1} : uint8_t{0};
    msg.between_pose_prev_curr = isoToPoseMsg(event.T_Bkm1_Bk);

    msg.interval_health.num_frames = event.interval_health.num_frames;
    msg.interval_health.num_pose_valid_frames = event.interval_health.num_pose_valid_frames;
    msg.interval_health.num_degraded_frames = event.interval_health.num_degraded_frames;
    msg.interval_health.num_lost_frames = event.interval_health.num_lost_frames;
    msg.interval_health.min_tracks = event.interval_health.min_tracks;
    msg.interval_health.mean_tracks = event.interval_health.mean_tracks;
    msg.interval_health.min_track_retention = event.interval_health.min_track_retention;
    msg.interval_health.mean_track_retention = event.interval_health.mean_track_retention;
    msg.interval_health.mean_pnp_inlier_ratio = event.interval_health.mean_pnp_inlier_ratio;
    msg.interval_health.max_pnp_reproj_rmse_px = event.interval_health.max_pnp_reproj_rmse_px;
    msg.interval_health.min_track_coverage = event.interval_health.min_track_coverage;
    msg.interval_health.mean_track_coverage = event.interval_health.mean_track_coverage;

    const auto count = event.ids.size();
    msg.track_ids.resize(count);
    msg.u_l.resize(count);
    msg.v_l.resize(count);
    msg.u_r.resize(count);
    msg.v_r.resize(count);
    msg.has_right.resize(count);

    for (size_t i = 0; i < count; ++i)
    {
        msg.track_ids[i] = event.ids[i];
        msg.u_l[i] = event.pl[i].x;
        msg.v_l[i] = event.pl[i].y;
        msg.u_r[i] = event.pr[i].x;
        msg.v_r[i] = event.pr[i].y;
        msg.has_right[i] = event.has_r[i];
    }

    msg.has_imu = event.has_imu ? uint8_t{1} : uint8_t{0};
    msg.pim_bytes = event.pim_bytes;
    return msg;
}

} // namespace visual_inertial::transport
