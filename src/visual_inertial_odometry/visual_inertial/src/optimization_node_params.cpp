#include <visual_inertial/optimization_node_params.hpp>

#include <Eigen/Geometry>

namespace visual_inertial
{

OptimizationNodeParamHandler::OptimizationNodeParamHandler(rclcpp::Node &node)
    : node_(node)
{
    declareNodeParams_();
    declareOptimizerParams_();
    validateOperationMode_();
    if (params_.node.localization_mode)
    {
        declareLocalizationParams_();
    }
}

const OptimizationNodeParams &OptimizationNodeParamHandler::params() const noexcept
{
    return params_;
}

void OptimizationNodeParamHandler::declareNodeParams_()
{
    auto &cfg = params_.node;

    cfg.keyframe_topic =
        node_.declare_parameter<std::string>("keyframe_topic", cfg.keyframe_topic);
    cfg.left_info_topic =
        node_.declare_parameter<std::string>("left_info_topic", cfg.left_info_topic);
    cfg.right_info_topic =
        node_.declare_parameter<std::string>("right_info_topic", cfg.right_info_topic);

    cfg.map_frame_id =
        node_.declare_parameter<std::string>("map_frame_id", cfg.map_frame_id);
    cfg.odom_frame_id =
        node_.declare_parameter<std::string>("odom_frame_id", cfg.odom_frame_id);
    cfg.body_frame_id =
        node_.declare_parameter<std::string>("body_frame_id", cfg.body_frame_id);

    cfg.auto_resolve_t_bc_from_tf =
        node_.declare_parameter<bool>("auto_resolve_t_bc_from_tf", cfg.auto_resolve_t_bc_from_tf);
    cfg.tf_pub_rate_hz =
        node_.declare_parameter<double>("tf_pub_rate_hz", cfg.tf_pub_rate_hz);
    cfg.max_keyframe_queue = static_cast<size_t>(
        node_.declare_parameter<int>("max_keyframe_queue", static_cast<int>(cfg.max_keyframe_queue)));

    cfg.operation_mode =
        node_.declare_parameter<std::string>("operation_mode", cfg.operation_mode);

    cfg.publish_optimization_result =
        node_.declare_parameter<bool>("publish_optimization_result", cfg.publish_optimization_result);
    cfg.optimization_result_topic =
        node_.declare_parameter<std::string>("optimization_result_topic", cfg.optimization_result_topic);
    cfg.publish_optimized_landmarks =
        node_.declare_parameter<bool>("publish_optimized_landmarks", cfg.publish_optimized_landmarks);
    cfg.landmark_topic =
        node_.declare_parameter<std::string>("landmark_topic", cfg.landmark_topic);
    cfg.lm_fetch_max = static_cast<size_t>(
        node_.declare_parameter<int>("lm_fetch_max", static_cast<int>(cfg.lm_fetch_max)));
}

void OptimizationNodeParamHandler::declareOptimizerParams_()
{
    auto &cfg = params_.optimizer;

    cfg.window_size = static_cast<size_t>(
        node_.declare_parameter<int>("window_size", static_cast<int>(cfg.window_size)));
    cfg.stereo_sigma_px =
        node_.declare_parameter<double>("stereo_sigma_px", cfg.stereo_sigma_px);
    cfg.stereo_huber_k =
        node_.declare_parameter<double>("stereo_huber_k", cfg.stereo_huber_k);

    cfg.prior_rot_sigma_rad =
        node_.declare_parameter<double>("prior_rot_sigma_rad", cfg.prior_rot_sigma_rad);
    cfg.prior_trans_sigma_m =
        node_.declare_parameter<double>("prior_trans_sigma_m", cfg.prior_trans_sigma_m);

    cfg.vel_prior_sigma =
        node_.declare_parameter<double>("vel_prior_sigma", cfg.vel_prior_sigma);
    cfg.bias_acc_prior_sigma =
        node_.declare_parameter<double>("bias_acc_prior_sigma", cfg.bias_acc_prior_sigma);
    cfg.bias_gyro_prior_sigma =
        node_.declare_parameter<double>("bias_gyro_prior_sigma", cfg.bias_gyro_prior_sigma);

    cfg.use_vo_between =
        node_.declare_parameter<bool>("use_vo_between", cfg.use_vo_between);
    cfg.between_rot_sigma_rad =
        node_.declare_parameter<double>("between_rot_sigma_rad", cfg.between_rot_sigma_rad);
    cfg.between_trans_sigma_m =
        node_.declare_parameter<double>("between_trans_sigma_m", cfg.between_trans_sigma_m);
    cfg.between_huber_k =
        node_.declare_parameter<double>("between_huber_k", cfg.between_huber_k);
    cfg.use_interval_health_for_vo_between = node_.declare_parameter<bool>(
        "use_interval_health_for_vo_between", cfg.use_interval_health_for_vo_between);
    cfg.between_health_min_pose_valid_fraction = node_.declare_parameter<double>(
        "between_health_min_pose_valid_fraction", cfg.between_health_min_pose_valid_fraction);
    cfg.between_health_min_track_retention = node_.declare_parameter<double>(
        "between_health_min_track_retention", cfg.between_health_min_track_retention);
    cfg.between_health_min_pnp_inlier_ratio = node_.declare_parameter<double>(
        "between_health_min_pnp_inlier_ratio", cfg.between_health_min_pnp_inlier_ratio);
    cfg.between_health_min_track_coverage = node_.declare_parameter<double>(
        "between_health_min_track_coverage", cfg.between_health_min_track_coverage);
    cfg.between_health_max_pnp_reproj_rmse_px = node_.declare_parameter<double>(
        "between_health_max_pnp_reproj_rmse_px", cfg.between_health_max_pnp_reproj_rmse_px);
    cfg.between_health_max_sigma_scale = node_.declare_parameter<double>(
        "between_health_max_sigma_scale", cfg.between_health_max_sigma_scale);
    cfg.between_health_skip_quality = node_.declare_parameter<double>(
        "between_health_skip_quality", cfg.between_health_skip_quality);

    cfg.use_imu =
        node_.declare_parameter<bool>("use_imu", cfg.use_imu);
    cfg.init_landmarks_from_stereo =
        node_.declare_parameter<bool>("init_landmarks_from_stereo", cfg.init_landmarks_from_stereo);
    cfg.prune_unobserved_landmarks =
        node_.declare_parameter<bool>("prune_unobserved_landmarks", cfg.prune_unobserved_landmarks);

    parseBodyCameraExtrinsics_();
}

void OptimizationNodeParamHandler::declareLocalizationParams_()
{
    auto localization = visual_inertial_localization::LocalizationConfig{};

    localization.tag_map_path =
        node_.declare_parameter<std::string>("localization_tag_map_path", "");
    localization.tag_topic =
        node_.declare_parameter<std::string>("tag_topic", localization.tag_topic);
    localization.tag_tf_lookup_timeout_ms =
        node_.declare_parameter<double>("tag_tf_lookup_timeout_ms", localization.tag_tf_lookup_timeout_ms);
    localization.tag_max_age_s =
        node_.declare_parameter<double>("localization_tag_max_age_s", localization.tag_max_age_s);
    localization.tag_buffer_age_s =
        node_.declare_parameter<double>("tag_buffer_age_s", localization.tag_buffer_age_s);
    localization.max_tag_hamming =
        node_.declare_parameter<int>("max_tag_hamming", localization.max_tag_hamming);
    localization.min_tag_decision_margin =
        node_.declare_parameter<double>("min_tag_decision_margin", localization.min_tag_decision_margin);
    localization.max_tag_range_m =
        node_.declare_parameter<double>("max_tag_range_m", localization.max_tag_range_m);
    localization.max_tag_oblique_angle_deg =
        node_.declare_parameter<double>("max_tag_oblique_angle_deg", localization.max_tag_oblique_angle_deg);
    localization.pose_prior_rot_sigma_rad =
        node_.declare_parameter<double>("localization_pose_prior_rot_sigma_rad", localization.pose_prior_rot_sigma_rad);
    localization.pose_prior_trans_sigma_m =
        node_.declare_parameter<double>("localization_pose_prior_trans_sigma_m", localization.pose_prior_trans_sigma_m);
    localization.pose_prior_huber_k =
        node_.declare_parameter<double>("localization_pose_prior_huber_k", localization.pose_prior_huber_k);
    localization.cluster_translation_m =
        node_.declare_parameter<double>("localization_cluster_translation_m", localization.cluster_translation_m);
    localization.cluster_rotation_deg =
        node_.declare_parameter<double>("localization_cluster_rotation_deg", localization.cluster_rotation_deg);
    localization.stable_hypothesis_age_s =
        node_.declare_parameter<double>("localization_stable_hypothesis_age_s", localization.stable_hypothesis_age_s);
    localization.stable_min_frames = static_cast<size_t>(
        node_.declare_parameter<int>("localization_stable_min_frames", static_cast<int>(localization.stable_min_frames)));
    localization.relocalization_min_history_frames = static_cast<size_t>(
        node_.declare_parameter<int>(
            "localization_relocalization_min_history_frames",
            static_cast<int>(localization.relocalization_min_history_frames)));
    localization.stable_translation_m =
        node_.declare_parameter<double>("localization_stable_translation_m", localization.stable_translation_m);
    localization.stable_rotation_deg =
        node_.declare_parameter<double>("localization_stable_rotation_deg", localization.stable_rotation_deg);
    localization.relocalize_translation_m =
        node_.declare_parameter<double>("localization_relocalize_translation_m", localization.relocalize_translation_m);
    localization.relocalize_rotation_deg =
        node_.declare_parameter<double>("localization_relocalize_rotation_deg", localization.relocalize_rotation_deg);
    localization.tracking_deadband_translation_m =
        node_.declare_parameter<double>(
            "localization_tracking_deadband_translation_m",
            localization.tracking_deadband_translation_m);
    localization.tracking_deadband_rotation_deg =
        node_.declare_parameter<double>(
            "localization_tracking_deadband_rotation_deg",
            localization.tracking_deadband_rotation_deg);

    params_.localization = std::move(localization);
    parseTagFrameOverrides_();
}

void OptimizationNodeParamHandler::parseBodyCameraExtrinsics_()
{
    auto &cfg = params_.optimizer;

    Eigen::Quaterniond q_default(cfg.T_BC.rotation());
    q_default.normalize();
    const auto t_bc = node_.declare_parameter<std::vector<double>>(
        "T_BC_translation",
        {cfg.T_BC.translation().x(), cfg.T_BC.translation().y(), cfg.T_BC.translation().z()});
    const auto q_bc_xyzw = node_.declare_parameter<std::vector<double>>(
        "T_BC_quaternion_xyzw",
        {q_default.x(), q_default.y(), q_default.z(), q_default.w()});

    if (t_bc.size() != 3 || q_bc_xyzw.size() != 4)
    {
        RCLCPP_WARN(
            node_.get_logger(),
            "T_BC params malformed (need 3 translation + 4 quaternion entries); using defaults");
        return;
    }

    Eigen::Quaterniond q(q_bc_xyzw[3], q_bc_xyzw[0], q_bc_xyzw[1], q_bc_xyzw[2]);
    if (q.norm() <= 1e-9)
    {
        RCLCPP_WARN(
            node_.get_logger(),
            "T_BC quaternion has near-zero norm; using default extrinsics");
        return;
    }

    q.normalize();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(t_bc[0], t_bc[1], t_bc[2]);
    cfg.T_BC = T;
}

void OptimizationNodeParamHandler::parseTagFrameOverrides_()
{
    if (!params_.localization.has_value())
    {
        return;
    }

    const auto tag_frame_ids = node_.declare_parameter<std::vector<int64_t>>(
        "tag_frame_ids", std::vector<int64_t>{});
    const auto tag_frame_names = node_.declare_parameter<std::vector<std::string>>(
        "tag_frame_names", std::vector<std::string>{});

    if (tag_frame_ids.size() != tag_frame_names.size())
    {
        RCLCPP_WARN(
            node_.get_logger(),
            "tag_frame_ids/tag_frame_names size mismatch (%zu vs %zu); ignoring overrides",
            tag_frame_ids.size(),
            tag_frame_names.size());
        return;
    }

    for (size_t i = 0; i < tag_frame_ids.size(); ++i)
    {
        params_.localization->tag_frame_overrides.emplace(
            static_cast<int>(tag_frame_ids[i]),
            tag_frame_names[i]);
    }
}

void OptimizationNodeParamHandler::validateOperationMode_()
{
    auto &cfg = params_.node;
    if (cfg.operation_mode != "mapping" && cfg.operation_mode != "localization")
    {
        RCLCPP_WARN(
            node_.get_logger(),
            "Unsupported operation_mode='%s'; falling back to mapping",
            cfg.operation_mode.c_str());
        cfg.operation_mode = "mapping";
    }
    cfg.localization_mode = (cfg.operation_mode == "localization");
}

} // namespace visual_inertial
