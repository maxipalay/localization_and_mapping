// KeyframePolicy.cpp
#include "visual_inertial_lib/keyframe_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

void KeyframePolicy::reset()
{
    has_last_kf_ = false;
    last_kf_id_ = 0;
    last_kf_t_s_ = 0.0;
    last_kf_T_WC_ = Eigen::Isometry3d::Identity();
    last_kf_ids_sorted_.clear();
}

KeyframePolicy::Decision KeyframePolicy::evaluate(const Input &in)
{
    Decision d;
    d.num_tracks = static_cast<int>(in.num_tracks);

    // First keyframe (or after reset)
    if (!has_last_kf_)
    {
        d.make_keyframe = true;
        d.reasons |= kInit;
        d.dt_since_last_kf_s = std::numeric_limits<double>::infinity();
        d.shared_tracks = 0;
        d.shared_ratio = 0.0;
        return d;
    }

    // Time since last KF
    d.dt_since_last_kf_s = in.t_s - last_kf_t_s_;
    if (d.dt_since_last_kf_s < 0.0)
    {
        // Clock went backwards or timestamps not monotonic; treat as 0 to avoid weirdness.
        d.dt_since_last_kf_s = 0.0;
    }

    d.shared_tracks = computeSharedWithLastKF(in.track_ids, in.num_tracks);

    const int last_kf_n = static_cast<int>(last_kf_ids_sorted_.size());
    d.shared_ratio = (last_kf_n > 0) ? (static_cast<double>(d.shared_tracks) / static_cast<double>(last_kf_n)) : 0.0;

    // Motion since last KF (using camera pose in world)
    d.trans_since_last_kf_m = 0.0;
    d.rot_since_last_kf_deg = 0.0;
    if (in.pose_valid)
    {
        const Eigen::Vector3d t_kf = last_kf_T_WC_.translation();
        const Eigen::Vector3d t_c = in.T_WC.translation();
        d.trans_since_last_kf_m = (t_c - t_kf).norm();

        const Eigen::Matrix3d R_kf = last_kf_T_WC_.linear();
        const Eigen::Matrix3d R_c = in.T_WC.linear();
        const Eigen::Matrix3d R_rel = R_kf.transpose() * R_c; // rotation from KF cam to current cam, expressed in KF
        d.rot_since_last_kf_deg = rotationAngleDeg(R_rel);
    }

    // --- Evaluate triggers ---
    bool want_kf = false;

    // Track count trigger (overall tracking health)
    if (cfg_.min_tracks > 0 && static_cast<int>(in.num_tracks) < cfg_.min_tracks)
    {
        d.reasons |= kLowTracks;
        want_kf = true;
    }

    // Shared tracks trigger (keyframe-to-keyframe constraint strength)
    bool low_shared = false;
    if (cfg_.min_shared_tracks > 0 && d.shared_tracks < cfg_.min_shared_tracks)
        low_shared = true;
    if (cfg_.min_shared_ratio >= 0.0 && last_kf_n > 0 && d.shared_ratio < cfg_.min_shared_ratio)
        low_shared = true;

    if (low_shared)
    {
        d.reasons |= kLowSharedTracks;
        want_kf = true;
    }

    // Motion triggers
    if (in.pose_valid)
    {
        if (cfg_.min_trans_m > 0.0 && d.trans_since_last_kf_m >= cfg_.min_trans_m)
        {
            d.reasons |= kMotionTranslation;
            want_kf = true;
        }
        if (cfg_.min_rot_deg > 0.0 && d.rot_since_last_kf_deg >= cfg_.min_rot_deg)
        {
            d.reasons |= kMotionRotation;
            want_kf = true;
        }
    }

    // Max interval (force keyframe)
    if (cfg_.max_kf_dt_s > 0.0 && d.dt_since_last_kf_s >= cfg_.max_kf_dt_s)
    {
        d.reasons |= kMaxInterval;
        if (cfg_.force_kf_on_max_interval)
        {
            d.make_keyframe = true;
            return d; // forced
        }
        want_kf = true;
    }

    // Min interval (rate limit)
    const bool min_interval_blocked =
        (cfg_.min_kf_dt_s > 0.0 && d.dt_since_last_kf_s < cfg_.min_kf_dt_s);

    if (want_kf)
    {
        if (!min_interval_blocked)
        {
            d.make_keyframe = true;
            return d;
        }

        // Blocked by min interval unless we allow early KF for quality drops
        if (cfg_.allow_early_kf_on_quality_drop)
        {
            const bool quality_drop =
                (d.reasons & (kLowTracks | kLowSharedTracks)) != 0;

            if (quality_drop)
            {
                d.make_keyframe = true;
                return d;
            }
        }

        d.reasons |= kMinIntervalBlock;
        d.make_keyframe = false;
        return d;
    }

    // No triggers
    d.make_keyframe = false;
    return d;
}

void KeyframePolicy::onKeyframeCreated(uint64_t kf_id,
                                       double t_s,
                                       const Eigen::Isometry3d &T_WC_kf,
                                       const TrackId *kf_track_ids,
                                       size_t num_kf_tracks)
{
    has_last_kf_ = true;
    last_kf_id_ = kf_id;
    last_kf_t_s_ = t_s;
    last_kf_T_WC_ = T_WC_kf;

    last_kf_ids_sorted_.clear();
    if (kf_track_ids && num_kf_tracks > 0)
    {
        last_kf_ids_sorted_.assign(kf_track_ids, kf_track_ids + num_kf_tracks);
        std::sort(last_kf_ids_sorted_.begin(), last_kf_ids_sorted_.end());
        last_kf_ids_sorted_.erase(
            std::unique(last_kf_ids_sorted_.begin(), last_kf_ids_sorted_.end()),
            last_kf_ids_sorted_.end());
    }
}

void KeyframePolicy::onKeyframeCreated(uint64_t kf_id,
                                       double t_s,
                                       const Eigen::Isometry3d &T_WC_kf,
                                       const std::vector<TrackId> &kf_track_ids)
{
    onKeyframeCreated(kf_id, t_s, T_WC_kf, kf_track_ids.data(), kf_track_ids.size());
}

double KeyframePolicy::rotationAngleDeg(const Eigen::Matrix3d &R)
{
    // Angle from rotation matrix trace: cos(theta) = (trace(R)-1)/2
    const double tr = R.trace();
    double c = 0.5 * (tr - 1.0);
    if (c > 1.0)
        c = 1.0;
    if (c < -1.0)
        c = -1.0;
    const double theta = std::acos(c);
    return theta * (180.0 / M_PI);
}

int KeyframePolicy::computeSharedWithLastKF(const TrackId *ids, size_t n) const
{
    if (!ids || n == 0)
        return 0;
    if (last_kf_ids_sorted_.empty())
        return 0;

    int shared = 0;
    for (size_t i = 0; i < n; ++i)
    {
        const TrackId id = ids[i];
        if (std::binary_search(last_kf_ids_sorted_.begin(), last_kf_ids_sorted_.end(), id))
        {
            ++shared;
        }
    }
    return shared;
}
