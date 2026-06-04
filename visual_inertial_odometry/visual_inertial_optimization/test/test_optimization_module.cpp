#include <gtest/gtest.h>

#include "visual_inertial_optimization/optimization.hpp"

#include <opencv2/core.hpp>

namespace
{

CameraRig makeValidRig()
{
    CameraRig rig;
    rig.left.cam_id = 0;
    rig.right.cam_id = 1;
    rig.left.K <<
        300.0, 0.0, 320.0,
        0.0, 300.0, 240.0,
        0.0, 0.0, 1.0;
    rig.right.K = rig.left.K;
    rig.baseline = 0.10;
    return rig;
}

OptimizationConfig makeBaseConfig()
{
    OptimizationConfig config;
    config.use_imu = false;
    config.use_vo_between = true;
    config.use_interval_health_for_vo_between = true;
    config.window_size = 4;
    config.rig = makeValidRig();
    return config;
}

KeyframeEvent makeStereoKeyframe(
    uint64_t kf_id,
    const FrontendIntervalHealth &interval_health,
    float disparity_px = 10.0F)
{
    KeyframeEvent kf;
    kf.kf_id = kf_id;
    kf.t_start = static_cast<double>(kf_id - 1);
    kf.t_end = static_cast<double>(kf_id);
    kf.T_OB = Eigen::Isometry3d::Identity();
    kf.interval_health = interval_health;
    kf.ids = {1};
    kf.pl = {cv::Point2f(320.0F, 240.0F)};
    kf.pr = {cv::Point2f(320.0F - disparity_px, 240.0F)};
    kf.has_r = {1};
    return kf;
}

FrontendIntervalHealth goodIntervalHealth()
{
    FrontendIntervalHealth health;
    health.num_frames = 5;
    health.num_pose_valid_frames = 5;
    health.min_track_retention = 0.9;
    health.mean_pnp_inlier_ratio = 0.8;
    health.max_pnp_reproj_rmse_px = 1.0;
    health.min_track_coverage = 0.9;
    return health;
}

} // namespace

TEST(OptimizationModuleTest, RejectsInvalidRigAndReportsReadiness)
{
    OptimizationConfig config;
    config.use_imu = false;
    OptimizationModule module(config);

    EXPECT_FALSE(module.ready());
    EXPECT_FALSE(module.initializeRig(CameraRig{}));
    EXPECT_FALSE(module.ready());
    EXPECT_EQ(std::nullopt, module.push(KeyframeEvent{}));
}

TEST(OptimizationModuleTest, InitializesWithValidRigAndProducesFirstResult)
{
    OptimizationModule module(makeBaseConfig());

    ASSERT_TRUE(module.initializeRig(makeValidRig()));
    EXPECT_TRUE(module.ready());

    const auto result = module.push(makeStereoKeyframe(1, goodIntervalHealth()));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->kf_id, 1U);
    EXPECT_GT(result->stats.num_stereo_factors_added, 0);
    EXPECT_EQ(result->stats.num_prior_factors_added, 3);
    EXPECT_FALSE(result->stats.had_vo_between_measurement);
}

TEST(OptimizationModuleTest, SkipsStereoWhenIntervalHealthHasNoPoseSupport)
{
    auto config = makeBaseConfig();
    config.between_health_skip_quality = 0.5;
    OptimizationModule module(config);
    ASSERT_TRUE(module.initializeRig(makeValidRig()));

    FrontendIntervalHealth health;
    health.num_frames = 5;
    health.num_pose_valid_frames = 0;
    health.num_lost_frames = 5;
    health.min_track_retention = 0.0;
    health.mean_pnp_inlier_ratio = 0.0;
    health.max_pnp_reproj_rmse_px = 10.0;
    health.min_track_coverage = 0.0;

    const auto result = module.push(makeStereoKeyframe(1, health));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->stats.num_stereo_factors_added, 0);
    EXPECT_EQ(result->stats.num_prior_factors_added, 3);
    EXPECT_FALSE(result->stats.had_vo_between_measurement);
}

TEST(OptimizationModuleTest, SkipsNewLandmarkStereoFactorWhenDisparityIsDegenerate)
{
    OptimizationModule module(makeBaseConfig());

    ASSERT_TRUE(module.initializeRig(makeValidRig()));

    const auto result = module.push(makeStereoKeyframe(1, goodIntervalHealth(), 0.0F));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->stats.num_stereo_factors_added, 0);
    EXPECT_EQ(result->stats.num_landmarks_created, 0);
    EXPECT_EQ(result->stats.num_prior_factors_added, 3);
}

TEST(OptimizationModuleTest, SkipsVoBetweenOnPoorIntervalHealthButKeepsStereo)
{
    auto config = makeBaseConfig();
    config.between_health_skip_quality = 0.7;
    OptimizationModule module(config);
    ASSERT_TRUE(module.initializeRig(makeValidRig()));
    ASSERT_TRUE(module.push(makeStereoKeyframe(1, goodIntervalHealth())).has_value());

    FrontendIntervalHealth poor_between_health;
    poor_between_health.num_frames = 5;
    poor_between_health.num_pose_valid_frames = 1;
    poor_between_health.num_degraded_frames = 4;
    poor_between_health.min_track_retention = 0.1;
    poor_between_health.mean_pnp_inlier_ratio = 0.1;
    poor_between_health.max_pnp_reproj_rmse_px = 8.0;
    poor_between_health.min_track_coverage = 0.1;

    AbsolutePosePrior prior;
    prior.T_WB = Eigen::Isometry3d::Identity();

    const auto result = module.push(
        makeStereoKeyframe(2, poor_between_health),
        Eigen::Isometry3d::Identity(),
        {prior});

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->stats.had_vo_between_measurement);
    EXPECT_TRUE(result->stats.skipped_vo_between_factor);
    EXPECT_FALSE(result->stats.used_vo_between_factor);
    EXPECT_GT(result->stats.num_stereo_factors_added, 0);
    EXPECT_LT(result->stats.vo_between_quality, config.between_health_skip_quality);
}
