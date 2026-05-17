#include <gtest/gtest.h>

#include "visual_inertial_frontend/keyframe_policy.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <vector>

namespace
{

Eigen::Isometry3d makePose(double x_m, double yaw_deg = 0.0)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = x_m;

    const double yaw_rad = yaw_deg * M_PI / 180.0;
    pose.linear() =
        Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return pose;
}

} // namespace

TEST(KeyframePolicyTest, FirstEvaluationRequestsInitialKeyframe)
{
    KeyframePolicy policy;

    const std::vector<KeyframePolicy::TrackId> track_ids{1, 2, 3};
    KeyframePolicy::Input in;
    in.t_s = 0.0;
    in.pose_valid = true;
    in.track_ids = track_ids.data();
    in.num_tracks = track_ids.size();
    in.num_pnp_tracks = track_ids.size();

    const auto decision = policy.evaluate(in);

    EXPECT_TRUE(decision.make_keyframe);
    EXPECT_NE(0u, decision.reasons & KeyframePolicy::kInit);
    EXPECT_TRUE(std::isinf(decision.dt_since_last_kf_s));
}

TEST(KeyframePolicyTest, MaxIntervalForcesKeyframe)
{
    KeyframePolicy::Config cfg;
    cfg.min_kf_dt_s = 0.2;
    cfg.max_kf_dt_s = 1.0;
    cfg.force_kf_on_max_interval = true;

    KeyframePolicy policy(cfg);
    const std::vector<KeyframePolicy::TrackId> last_ids{1, 2, 3, 4};
    policy.onKeyframeCreated(7, 0.0, makePose(0.0), last_ids);

    KeyframePolicy::Input in;
    in.t_s = 1.25;
    in.T_WC = makePose(0.0);
    in.pose_valid = true;
    in.track_ids = last_ids.data();
    in.num_tracks = last_ids.size();
    in.num_pnp_tracks = last_ids.size();

    const auto decision = policy.evaluate(in);

    EXPECT_TRUE(decision.make_keyframe);
    EXPECT_NE(0u, decision.reasons & KeyframePolicy::kMaxInterval);
}

TEST(KeyframePolicyTest, MinIntervalBlocksPureMotionTrigger)
{
    KeyframePolicy::Config cfg;
    cfg.min_kf_dt_s = 1.0;
    cfg.max_kf_dt_s = 10.0;
    cfg.min_trans_m = 0.2;
    cfg.min_rot_deg = 10.0;
    cfg.allow_early_kf_on_quality_drop = true;
    cfg.min_tracks = 0;
    cfg.min_pnp_tracks = 0;
    cfg.min_shared_tracks = 0;
    cfg.min_shared_ratio = -1.0;

    KeyframePolicy policy(cfg);
    const std::vector<KeyframePolicy::TrackId> last_ids{1, 2, 3, 4};
    policy.onKeyframeCreated(8, 0.0, makePose(0.0), last_ids);

    KeyframePolicy::Input in;
    in.t_s = 0.25;
    in.T_WC = makePose(0.3);
    in.pose_valid = true;
    in.track_ids = last_ids.data();
    in.num_tracks = last_ids.size();
    in.num_pnp_tracks = last_ids.size();

    const auto decision = policy.evaluate(in);

    EXPECT_FALSE(decision.make_keyframe);
    EXPECT_NE(0u, decision.reasons & KeyframePolicy::kMotionTranslation);
    EXPECT_NE(0u, decision.reasons & KeyframePolicy::kMinIntervalBlock);
}

TEST(KeyframePolicyTest, QualityDropCanBypassMinInterval)
{
    KeyframePolicy::Config cfg;
    cfg.min_kf_dt_s = 1.0;
    cfg.max_kf_dt_s = 10.0;
    cfg.min_tracks = 5;
    cfg.min_pnp_tracks = 0;
    cfg.min_shared_tracks = 0;
    cfg.min_shared_ratio = -1.0;
    cfg.allow_early_kf_on_quality_drop = true;

    KeyframePolicy policy(cfg);
    const std::vector<KeyframePolicy::TrackId> last_ids{1, 2, 3, 4};
    policy.onKeyframeCreated(9, 0.0, makePose(0.0), last_ids);

    const std::vector<KeyframePolicy::TrackId> current_ids{1, 2, 3};
    KeyframePolicy::Input in;
    in.t_s = 0.25;
    in.T_WC = makePose(0.0);
    in.pose_valid = true;
    in.track_ids = current_ids.data();
    in.num_tracks = current_ids.size();
    in.num_pnp_tracks = current_ids.size();

    const auto decision = policy.evaluate(in);

    EXPECT_TRUE(decision.make_keyframe);
    EXPECT_NE(0u, decision.reasons & KeyframePolicy::kLowTracks);
    EXPECT_EQ(0u, decision.reasons & KeyframePolicy::kMinIntervalBlock);
}
