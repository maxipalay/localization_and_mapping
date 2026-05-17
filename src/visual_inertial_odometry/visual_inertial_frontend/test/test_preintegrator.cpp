#include <gtest/gtest.h>

#include "visual_inertial_frontend/preintegrator.hpp"

#include <Eigen/Core>

TEST(PreintegratorTest, InvalidIntervalIsRejected)
{
    ImuPreintegrator preintegrator;

    const auto result = preintegrator.buildAndConsume(1, 1.0, 2, 1.0);

    EXPECT_EQ(ImuPreintegrator::BuildStatus::kInvalidInterval, result.status);
    EXPECT_FALSE(result.ok());
}

TEST(PreintegratorTest, RequiresCoverageToIntervalEnd)
{
    ImuPreintegrator preintegrator;
    preintegrator.push(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    preintegrator.push(0.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    const auto result = preintegrator.buildAndConsume(1, 0.0, 2, 0.2);

    EXPECT_EQ(ImuPreintegrator::BuildStatus::kNoCoverageToT1, result.status);
    EXPECT_FALSE(result.ok());
}

TEST(PreintegratorTest, RequiresAnchorAtOrBeforeIntervalStart)
{
    ImuPreintegrator preintegrator;
    preintegrator.push(0.2, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    preintegrator.push(0.3, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    const auto result = preintegrator.buildAndConsume(1, 0.1, 2, 0.25);

    EXPECT_EQ(ImuPreintegrator::BuildStatus::kNoAnchorAtOrBeforeT0, result.status);
    EXPECT_FALSE(result.ok());
}

TEST(PreintegratorTest, BuildAndConsumeProducesPacketAndKeepsAnchor)
{
    ImuPreintegratorConfig cfg;
    cfg.keep_anchor = true;

    ImuPreintegrator preintegrator(cfg);
    preintegrator.push(0.0, Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero());
    preintegrator.push(0.1, Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero());
    preintegrator.push(0.2, Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero());

    const auto result = preintegrator.buildAndConsume(10, 0.0, 11, 0.15);

    ASSERT_TRUE(result.ok());
    EXPECT_EQ(ImuPreintegrator::BuildStatus::kSuccess, result.status);
    EXPECT_TRUE(result.packet.valid);
    EXPECT_EQ(10u, result.packet.kf_id0);
    EXPECT_EQ(11u, result.packet.kf_id1);
    EXPECT_DOUBLE_EQ(0.0, result.packet.t0_s);
    EXPECT_DOUBLE_EQ(0.15, result.packet.t1_s);
    EXPECT_FALSE(result.packet.bytes.empty());

    EXPECT_EQ(2u, preintegrator.size());
    EXPECT_DOUBLE_EQ(0.1, preintegrator.oldestTime());
    EXPECT_DOUBLE_EQ(0.2, preintegrator.newestTime());
}
