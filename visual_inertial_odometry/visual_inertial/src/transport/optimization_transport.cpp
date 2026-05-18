#include "visual_inertial/transport/optimization_transport.hpp"

#include "visual_inertial/transport/common_transport.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace visual_inertial::transport
{

KeyframeEvent toKeyframeEvent(const visual_inertial::msg::Keyframe &msg)
{
    KeyframeEvent event;
    event.kf_id = msg.kf_id;
    event.t_start = msg.t_start;
    event.t_end = msg.t_end;
    event.T_OB = poseMsgToIso(msg.pose_odom_body);
    event.has_vo_between = (msg.has_vo_between != 0);
    if (event.has_vo_between)
    {
        event.T_Bkm1_Bk = poseMsgToIso(msg.between_pose_prev_curr);
    }

    event.interval_health.num_frames = msg.interval_health.num_frames;
    event.interval_health.num_pose_valid_frames = msg.interval_health.num_pose_valid_frames;
    event.interval_health.num_degraded_frames = msg.interval_health.num_degraded_frames;
    event.interval_health.num_lost_frames = msg.interval_health.num_lost_frames;
    event.interval_health.min_tracks = msg.interval_health.min_tracks;
    event.interval_health.mean_tracks = msg.interval_health.mean_tracks;
    event.interval_health.min_track_retention = msg.interval_health.min_track_retention;
    event.interval_health.mean_track_retention = msg.interval_health.mean_track_retention;
    event.interval_health.mean_pnp_inlier_ratio = msg.interval_health.mean_pnp_inlier_ratio;
    event.interval_health.max_pnp_reproj_rmse_px = msg.interval_health.max_pnp_reproj_rmse_px;
    event.interval_health.min_track_coverage = msg.interval_health.min_track_coverage;
    event.interval_health.mean_track_coverage = msg.interval_health.mean_track_coverage;

    const size_t count = msg.track_ids.size();
    if (msg.u_l.size() != count || msg.v_l.size() != count ||
        msg.u_r.size() != count || msg.v_r.size() != count ||
        msg.has_right.size() != count)
    {
        event.ids.clear();
        event.pl.clear();
        event.pr.clear();
        event.has_r.clear();
        return event;
    }

    event.ids.resize(count);
    event.pl.resize(count);
    event.pr.resize(count);
    event.has_r.resize(count);
    for (size_t i = 0; i < count; ++i)
    {
        event.ids[i] = msg.track_ids[i];
        event.pl[i] = cv::Point2f(msg.u_l[i], msg.v_l[i]);
        event.pr[i] = cv::Point2f(msg.u_r[i], msg.v_r[i]);
        event.has_r[i] = msg.has_right[i];
    }

    event.has_imu = (msg.has_imu != 0);
    event.pim_bytes = msg.pim_bytes;
    return event;
}

visual_inertial::msg::ImuBias makeImuBiasMsg(
    const visual_inertial::msg::Keyframe &msg,
    const OptimizationResult &result)
{
    visual_inertial::msg::ImuBias out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = "base_link";
    out.kf_id = result.kf_id;
    out.accel_bias.x = result.bias_opt.accel.x();
    out.accel_bias.y = result.bias_opt.accel.y();
    out.accel_bias.z = result.bias_opt.accel.z();
    out.gyro_bias.x = result.bias_opt.gyro.x();
    out.gyro_bias.y = result.bias_opt.gyro.y();
    out.gyro_bias.z = result.bias_opt.gyro.z();
    return out;
}

visual_inertial::msg::OptimizationResult makeOptimizationResultMsg(
    const visual_inertial::msg::Keyframe &msg,
    const OptimizationResult &result,
    const std::string &map_frame_id)
{
    visual_inertial::msg::OptimizationResult out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = map_frame_id;
    out.kf_id = result.kf_id;
    out.t_s = result.t_s;
    out.pose_wc_opt = isoToPoseMsg(result.T_WC_opt);
    out.pose_wb_opt = isoToPoseMsg(result.T_WB_opt);
    out.active_keyframe_poses.reserve(result.active_keyframe_poses.size());

    for (const auto &active_pose : result.active_keyframe_poses)
    {
        visual_inertial::msg::OptimizedKeyframePose active_pose_msg;
        active_pose_msg.kf_id = active_pose.kf_id;
        active_pose_msg.pose_wc_opt = isoToPoseMsg(active_pose.T_WC_opt);
        active_pose_msg.pose_wb_opt = isoToPoseMsg(active_pose.T_WB_opt);
        out.active_keyframe_poses.push_back(std::move(active_pose_msg));
    }

    out.stats.num_keyframes_in_window = result.stats.num_keyframes_in_window;
    out.stats.num_landmarks_alive = result.stats.num_landmarks_alive;
    out.stats.num_landmarks_created = result.stats.num_landmarks_created;
    out.stats.num_stereo_factors_added = result.stats.num_stereo_factors_added;
    out.stats.num_imu_factors_added = result.stats.num_imu_factors_added;
    out.stats.num_between_factors_added = result.stats.num_between_factors_added;
    out.stats.num_prior_factors_added = result.stats.num_prior_factors_added;
    out.stats.had_vo_between_measurement = result.stats.had_vo_between_measurement;
    out.stats.used_vo_between_factor = result.stats.used_vo_between_factor;
    out.stats.skipped_vo_between_factor = result.stats.skipped_vo_between_factor;
    out.stats.vo_between_quality = result.stats.vo_between_quality;
    out.stats.vo_between_sigma_scale = result.stats.vo_between_sigma_scale;
    out.stats.imu_only_update = result.stats.imu_only_update;
    out.stats.update_iterations = result.stats.update_iterations;
    out.stats.update_intermediate_steps = result.stats.update_intermediate_steps;
    out.stats.update_nonlinear_variables = result.stats.update_nonlinear_variables;
    out.stats.update_linear_variables = result.stats.update_linear_variables;
    out.stats.final_error = result.stats.final_error;
    out.stats.has_error_before = result.stats.has_error_before;
    out.stats.error_before = result.stats.error_before;
    out.stats.has_error_after = result.stats.has_error_after;
    out.stats.error_after = result.stats.error_after;
    out.stats.variables_relinearized = result.stats.variables_relinearized;
    out.stats.variables_reeliminated = result.stats.variables_reeliminated;
    out.stats.factors_recalculated = result.stats.factors_recalculated;
    out.stats.cliques = result.stats.cliques;
    out.has_velocity = result.has_velocity;
    out.velocity_opt.x = result.velocity_opt.x();
    out.velocity_opt.y = result.velocity_opt.y();
    out.velocity_opt.z = result.velocity_opt.z();
    out.accel_bias.x = result.bias_opt.accel.x();
    out.accel_bias.y = result.bias_opt.accel.y();
    out.accel_bias.z = result.bias_opt.accel.z();
    out.gyro_bias.x = result.bias_opt.gyro.x();
    out.gyro_bias.y = result.bias_opt.gyro.y();
    out.gyro_bias.z = result.bias_opt.gyro.z();
    out.has_pose_wb_covariance = result.has_pose_wb_covariance;
    out.pose_wb_covariance = result.pose_wb_covariance;
    out.has_velocity_covariance = result.has_velocity_covariance;
    out.velocity_covariance = result.velocity_covariance;
    out.has_bias_covariance = result.has_bias_covariance;
    out.bias_covariance = result.bias_covariance;
    return out;
}

visual_inertial::msg::LocalizationFeedback makeLocalizationFeedbackMsg(
    const visual_inertial::msg::Keyframe &msg,
    const OptimizationResult &result)
{
    visual_inertial::msg::LocalizationFeedback out;
    out.header = msg.header;
    out.kf_id = result.kf_id;

    const Eigen::Isometry3d transform_odom_body = poseMsgToIso(msg.pose_odom_body);
    const Eigen::Isometry3d transform_map_body = result.T_WB_opt;
    out.pose_map_odom = isoToPoseMsg(transform_map_body * transform_odom_body.inverse());
    return out;
}

sensor_msgs::msg::PointCloud2 makeLandmarkPointCloudMsg(
    const std::vector<LandmarkEstimate> &landmarks,
    const rclcpp::Time &stamp,
    const std::string &frame_id)
{
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;
    out.height = 1;
    out.width = static_cast<uint32_t>(landmarks.size());
    out.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(landmarks.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

    for (const auto &landmark : landmarks)
    {
        *it_x = static_cast<float>(landmark.p_W.x());
        *it_y = static_cast<float>(landmark.p_W.y());
        *it_z = static_cast<float>(landmark.p_W.z());
        ++it_x;
        ++it_y;
        ++it_z;
    }

    return out;
}

} // namespace visual_inertial::transport
