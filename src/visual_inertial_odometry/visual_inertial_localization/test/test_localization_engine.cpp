#include "localization_test_utils.hpp"

#include <gtest/gtest.h>
#include <visual_inertial_localization/localization.hpp>

namespace viloc_test = visual_inertial_localization::test;
using visual_inertial_localization::LocalizationConfig;
using visual_inertial_localization::LocalizationModule;

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
    return config;
}

TEST(LocalizationEngineTest, LoadTagMapParsesValidTagsAndIgnoresInvalidEntries)
{
    const viloc_test::TempTagMapFile tag_map(
        "tags:\n"
        "  - id: 1\n"
        "    position: [0.0, 0.0, 0.0]\n"
        "    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]\n"
        "  - id: 2\n"
        "    position: [1.0, 2.0]\n"
        "    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]\n"
        "  - id: 3\n"
        "    position: [0.0, 0.0, 0.0]\n"
        "    orientation_xyzw: [0.0, 0.0, 0.0, 0.0]\n");

    LocalizationModule module(makeBaseConfig(tag_map.path().string()));
    const auto report = module.loadTagMap();

    EXPECT_TRUE(report.ok);
    EXPECT_EQ(report.mapped_tag_count, 1U);
    EXPECT_EQ(report.message, "loaded");
}

TEST(LocalizationEngineTest, IngestDetectionsRejectsLowDecisionMargin)
{
    const viloc_test::TempTagMapFile tag_map(viloc_test::singleTagMapYaml());
    auto config = makeBaseConfig(tag_map.path().string());
    config.min_tag_decision_margin = 40.0;

    LocalizationModule module(config);
    ASSERT_TRUE(module.loadTagMap().ok);

    auto tf_buffer = viloc_test::makeTfBuffer();
    viloc_test::setStaticTransform(
        tf_buffer, "body", "tag36h11:1", Eigen::Vector3d(0.0, 0.0, 1.0));
    viloc_test::setStaticTransform(
        tf_buffer, "odom", "body", Eigen::Vector3d::Zero());

    const auto report = module.ingestDetections(
        viloc_test::makeDetectionArray(0.0, 10.0),
        "body",
        "odom",
        tf_buffer);

    EXPECT_EQ(report.total_detections, 1U);
    EXPECT_EQ(report.accepted, 0U);
    EXPECT_EQ(report.skipped_margin, 1U);
    EXPECT_EQ(report.buffered, 0U);
}

TEST(LocalizationEngineTest, StableCorrectionRequiresConfiguredHistory)
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

    EXPECT_FALSE(module.estimateStableCorrection(rclcpp::Time(viloc_test::stampNs(0.1))).has_value());

    module.ingestDetections(viloc_test::makeDetectionArray(0.2, 50.0), "body", "odom", tf_buffer);

    const auto correction = module.estimateStableCorrection(rclcpp::Time(viloc_test::stampNs(0.2)));
    ASSERT_TRUE(correction.has_value());
    EXPECT_EQ(correction->frame_support, 3U);
    EXPECT_EQ(correction->support_count, 1U);
    EXPECT_NEAR(correction->T_MO.translation().z(), -1.0, 1e-9);
}

} // namespace
