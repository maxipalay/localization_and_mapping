#pragma once

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <visual_inertial_localization/localization.hpp>
#include <visual_inertial_optimization/optimization.hpp>

namespace visual_inertial
{

struct OptimizationNodeConfig
{
    std::string keyframe_topic{"keyframes"};
    std::string left_info_topic{"oak/left/camera_info"};
    std::string right_info_topic{"oak/right/camera_info"};
    std::string body_frame_id{"body"};
    std::string map_frame_id{"map"};
    std::string odom_frame_id{"odom"};
    std::string operation_mode{"mapping"};
    bool localization_mode{false};
    bool auto_resolve_t_bc_from_tf{true};
    double tf_pub_rate_hz{30.0};
    size_t max_keyframe_queue{30};
    bool publish_optimization_result{true};
    std::string optimization_result_topic{"optimization_result"};
    bool publish_optimized_landmarks{true};
    std::string landmark_topic{"optimized_landmarks"};
    size_t lm_fetch_max{0};
};

struct OptimizationNodeParams
{
    OptimizationNodeConfig node;
    OptimizationConfig optimizer;
    std::optional<visual_inertial_localization::LocalizationConfig> localization;
};

class OptimizationNodeParamHandler
{
public:
    explicit OptimizationNodeParamHandler(rclcpp::Node &node);

    const OptimizationNodeParams &params() const noexcept;

private:
    void declareNodeParams_();
    void declareOptimizerParams_();
    void declareLocalizationParams_();
    void parseBodyCameraExtrinsics_();
    void parseTagFrameOverrides_();
    void validateOperationMode_();

    rclcpp::Node &node_;
    OptimizationNodeParams params_;
};

} // namespace visual_inertial
