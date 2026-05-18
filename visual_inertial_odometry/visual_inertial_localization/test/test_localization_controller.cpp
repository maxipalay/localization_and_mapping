#include "localization_test_utils.hpp"

#include <gtest/gtest.h>
#include <visual_inertial_localization/localization.hpp>

using visual_inertial_localization::LocalizationAction;
using visual_inertial_localization::LocalizationConfig;
using visual_inertial_localization::LocalizationModule;
using visual_inertial_localization::LocalizationState;
namespace viloc_test = visual_inertial_localization::test;

namespace
{

LocalizationConfig makeBaseConfig(const std::string &tag_map_path)
{
    LocalizationConfig config;
    config.tag_map_path = tag_map_path;
    config.min_tag_decision_margin = 5.0;
    config.tag_tf_lookup_timeout_ms = 0.0;
    config.max_tag_oblique_angle_deg = 0.0;
    config.max_tag_range_m = 10.0;
    config.tag_max_age_s = 10.0;
    config.tag_buffer_age_s = 10.0;
    config.stable_hypothesis_age_s = 10.0;
    config.stable_min_frames = 3;
    config.relocalization_min_history_frames = 3;
    config.relocalize_translation_m = 0.1;
    config.relocalize_rotation_deg = 5.0;
    return config;
}

TEST(LocalizationControllerTest, ProcessKeyframeWaitsForBootstrapWithoutLocalizationEvidence)
{
    const viloc_test::TempTagMapFile tag_map(viloc_test::singleTagMapYaml());
    LocalizationModule module(makeBaseConfig(tag_map.path().string()));
    ASSERT_TRUE(module.loadTagMap().ok);

    const auto decision = module.processKeyframe(
        viloc_test::stampNs(0.0),
        Eigen::Isometry3d::Identity());

    EXPECT_EQ(decision.state, LocalizationState::OdomOnly);
    EXPECT_EQ(decision.action, LocalizationAction::None);
    EXPECT_TRUE(decision.waiting_for_bootstrap);
    EXPECT_FALSE(decision.optimizer_inputs.T_WB_init_override.has_value());
    EXPECT_TRUE(decision.optimizer_inputs.absolute_pose_priors.empty());
}

TEST(LocalizationControllerTest, ProcessKeyframeBootstrapsAndPopulatesOptimizerInputs)
{
    const viloc_test::TempTagMapFile tag_map(viloc_test::singleTagMapYaml());
    LocalizationModule module(makeBaseConfig(tag_map.path().string()));
    ASSERT_TRUE(module.loadTagMap().ok);

    auto tf_buffer = viloc_test::makeTfBuffer();
    viloc_test::setStaticTransform(
        tf_buffer, "body", "tag36h11:1", Eigen::Vector3d(0.0, 0.0, 1.0));
    viloc_test::setStaticTransform(
        tf_buffer, "odom", "body", Eigen::Vector3d::Zero());

    module.ingestDetections(viloc_test::makeDetectionArray(0.0, 50.0), "body", "odom", tf_buffer);
    module.ingestDetections(viloc_test::makeDetectionArray(0.1, 50.0), "body", "odom", tf_buffer);
    module.ingestDetections(viloc_test::makeDetectionArray(0.2, 50.0), "body", "odom", tf_buffer);

    const auto decision = module.processKeyframe(
        viloc_test::stampNs(0.2),
        Eigen::Isometry3d::Identity());

    EXPECT_EQ(decision.state, LocalizationState::Localized);
    EXPECT_EQ(decision.action, LocalizationAction::Bootstrap);
    EXPECT_FALSE(decision.waiting_for_bootstrap);
    ASSERT_TRUE(decision.bootstrap.has_value());
    EXPECT_EQ(decision.bootstrap->support_count, 1U);
    ASSERT_TRUE(decision.optimizer_inputs.T_WB_init_override.has_value());
    ASSERT_TRUE(decision.optimizer_inputs.T_WB_anchor_override.has_value());
    EXPECT_NEAR(
        decision.optimizer_inputs.T_WB_init_override->translation().z(),
        -1.0,
        1e-9);
}

TEST(LocalizationControllerTest, ProcessKeyframeRelocalizesWhenStableCorrectionDisagreesWithCurrentMapOdom)
{
    const viloc_test::TempTagMapFile tag_map(viloc_test::singleTagMapYaml());
    LocalizationModule module(makeBaseConfig(tag_map.path().string()));
    ASSERT_TRUE(module.loadTagMap().ok);

    auto tf_buffer = viloc_test::makeTfBuffer();
    viloc_test::setStaticTransform(
        tf_buffer, "body", "tag36h11:1", Eigen::Vector3d(0.0, 0.0, 1.0));
    viloc_test::setStaticTransform(
        tf_buffer, "odom", "body", Eigen::Vector3d::Zero());

    module.ingestDetections(viloc_test::makeDetectionArray(0.0, 50.0), "body", "odom", tf_buffer);
    module.ingestDetections(viloc_test::makeDetectionArray(0.1, 50.0), "body", "odom", tf_buffer);
    module.ingestDetections(viloc_test::makeDetectionArray(0.2, 50.0), "body", "odom", tf_buffer);
    ASSERT_EQ(
        module.processKeyframe(viloc_test::stampNs(0.2), Eigen::Isometry3d::Identity()).action,
        LocalizationAction::Bootstrap);

    module.updateMapOdomEstimate(Eigen::Isometry3d::Identity());

    module.ingestDetections(viloc_test::makeDetectionArray(0.3, 50.0), "body", "odom", tf_buffer);
    module.ingestDetections(viloc_test::makeDetectionArray(0.4, 50.0), "body", "odom", tf_buffer);
    module.ingestDetections(viloc_test::makeDetectionArray(0.5, 50.0), "body", "odom", tf_buffer);

    const auto decision = module.processKeyframe(
        viloc_test::stampNs(0.5),
        Eigen::Isometry3d::Identity());

    EXPECT_EQ(decision.state, LocalizationState::Localized);
    EXPECT_EQ(decision.action, LocalizationAction::Relocalize);
    ASSERT_TRUE(decision.relocalization.has_value());
    EXPECT_GE(decision.relocalization->frame_support, 3U);
    EXPECT_GT(decision.relocalization->translation_error_m, 0.9);
    ASSERT_TRUE(decision.optimizer_inputs.T_WB_init_override.has_value());
    EXPECT_NEAR(
        decision.optimizer_inputs.T_WB_init_override->translation().z(),
        -1.0,
        1e-9);
}

} // namespace
