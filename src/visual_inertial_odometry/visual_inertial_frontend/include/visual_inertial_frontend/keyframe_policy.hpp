#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <vector>

class KeyframePolicy
{
public:
    using TrackId = uint32_t;

    // Why we decided (bitmask so you can log/plot later).
    enum Reason : uint32_t
    {
        kNone = 0,
        kInit = 1u << 0,              // first keyframe / policy not initialized
        kMaxInterval = 1u << 1,       // exceeded max time since last KF (force)
        kMinIntervalBlock = 1u << 2,  // would like KF but blocked by min interval
        kLowTracks = 1u << 3,         // too few current tracks (quality drop)
        kLowSharedTracks = 1u << 4,   // too few tracks shared w/ last KF
        kMotionTranslation = 1u << 5, // exceeded translation threshold since last KF
        kMotionRotation = 1u << 6,    // exceeded rotation threshold since last KF
        kLowPnPTracks = 1u << 7,      // too few anchored tracks available for PnP
    };

    struct Config
    {
        // Rate control
        double min_kf_dt_s = 0.20; // don't spam KFs faster than this (unless forced)
        double max_kf_dt_s = 2.00; // force a KF if longer than this since last KF

        // Motion triggers (since last keyframe pose)
        double min_trans_m = 0.20; // KF if translation since last KF exceeds this
        double min_rot_deg = 10.0; // KF if rotation since last KF exceeds this

        // Track quality triggers
        int min_tracks = 120;           // if current tracked points drop below this -> KF (or reset behavior)
        int min_shared_tracks = 60;     // if shared tracks w/ last KF drop below this -> KF
        double min_shared_ratio = 0.25; // OR ratio shared/last_kf_tracks below this -> KF (set <0 to disable)
        int min_pnp_tracks = 80;        // if anchored/PnP-eligible tracks drop below this -> KF

        // Behavior knobs
        bool force_kf_on_max_interval = true;       // typical: true
        bool allow_early_kf_on_quality_drop = true; // allow KF even if min_kf_dt_s not satisfied, when tracking quality is bad
    };

    struct Input
    {
        double t_s = 0.0;

        // Current pose estimate (camera pose in world coordinates):
        Eigen::Isometry3d T_WC = Eigen::Isometry3d::Identity();
        bool pose_valid = false; // if false, motion triggers are ignored

        // Current tracks (deterministic order not required here)
        const TrackId *track_ids = nullptr;
        size_t num_tracks = 0;
        size_t num_pnp_tracks = 0;
    };

    struct Decision
    {
        bool make_keyframe = false;
        uint32_t reasons = kNone;

        // debug values
        double dt_since_last_kf_s = 0.0;
        double trans_since_last_kf_m = 0.0;
        double rot_since_last_kf_deg = 0.0;
        int num_tracks = 0;
        int num_pnp_tracks = 0;
        int shared_tracks = 0;
        double shared_ratio = 0.0;
    };

public:
    KeyframePolicy(const Config &cfg)
        : cfg_(cfg) {}

    KeyframePolicy() : cfg_(Config{}) {}

    const Config &config() const { return cfg_; }
    void setConfig(const Config &cfg) { cfg_ = cfg; }

    // Clears internal "last keyframe" state.
    void reset();

    uint64_t lastKeyframeId() const { return last_kf_id_; } // 0 means "none yet"
    bool hasLastKeyframe() const { return has_last_kf_; }

    // Evaluate whether we should create a keyframe *now*.
    // If it returns make_keyframe=true, you should typically call onKeyframeCreated(...)
    // after you actually create/commit the keyframe.
    Decision evaluate(const Input &in);

    // Notify the policy that a keyframe was created/accepted.
    // Provide the ids you actually stored in the keyframe (for overlap decisions).
    void onKeyframeCreated(uint64_t kf_id, double t_s,
                           const Eigen::Isometry3d &T_WC_kf,
                           const TrackId *kf_track_ids,
                           size_t num_kf_tracks);

    // Convenience overload if you store ids in a vector.
    void onKeyframeCreated(uint64_t kf_id, double t_s,
                           const Eigen::Isometry3d &T_WC_kf,
                           const std::vector<TrackId> &kf_track_ids);

private:
    // Helpers (implemented in .cpp)
    static double rotationAngleDeg(const Eigen::Matrix3d &R);
    int computeSharedWithLastKF(const TrackId *ids, size_t n) const;

private:
    Config cfg_;

    bool has_last_kf_ = false;
    uint64_t last_kf_id_ = 0; // <- store external KF id here

    double last_kf_t_s_ = 0.0;
    Eigen::Isometry3d last_kf_T_WC_ = Eigen::Isometry3d::Identity();

    // Stored ids from last keyframe for overlap computation
    // (kept sorted for fast intersection in computeSharedWithLastKF)
    std::vector<TrackId> last_kf_ids_sorted_;
};

// Returns a human-readable list of reasons in `reasons` (bitmask).
inline std::string keyframeReasonsToString(uint32_t reasons) {
  using R = KeyframePolicy::Reason;

  if (reasons == R::kNone) return "None";

  std::ostringstream oss;
  bool first = true;

  auto add = [&](const char* s) {
    if (!first) oss << " | ";
    oss << s;
    first = false;
  };

  if (reasons & R::kInit)              add("Init");
  if (reasons & R::kMaxInterval)       add("MaxInterval");
  if (reasons & R::kMinIntervalBlock)  add("MinIntervalBlock");
  if (reasons & R::kLowTracks)         add("LowTracks");
  if (reasons & R::kLowSharedTracks)   add("LowSharedTracks");
  if (reasons & R::kLowPnPTracks)      add("LowPnPTracks");
  if (reasons & R::kMotionTranslation) add("MotionTranslation");
  if (reasons & R::kMotionRotation)    add("MotionRotation");

  return oss.str();
}

inline std::string keyframeDecisionDebugLine(const KeyframePolicy::Decision &d)
{
    std::ostringstream oss;
    oss << "[KF] make=" << (d.make_keyframe ? 1 : 0)
        << " reasons=" << keyframeReasonsToString(d.reasons)
        << " dt=" << d.dt_since_last_kf_s
        << " trans=" << d.trans_since_last_kf_m
        << " rot_deg=" << d.rot_since_last_kf_deg
        << " tracks=" << d.num_tracks
        << " pnp_tracks=" << d.num_pnp_tracks
        << " shared=" << d.shared_tracks
        << " shared_ratio=" << d.shared_ratio;

    return oss.str();
}
