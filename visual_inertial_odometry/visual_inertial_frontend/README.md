# visual_inertial_frontend

## Overview

Stereo tracking and IMU processing library for the visual inertial stack.

<p align="center">
  <img src="./docs/Tracking Frontend.png" alt="visual_inertial_frontend overview" width="720" style="border-radius: 12px;" />
</p>

The goal of this package is to take in sensor inputs (rectified stereo images + IMU) and produce odometry estimates + keyframes (referred to as KFs).

As implemented right now, odometry estimates are calculated exclusively from visual odometry. The module produces an absolute (since inception) transform and a relative transform (between frames). These are produced at camera rate/on every frame.

Keyframes are a snapshot of all the events and important information within a timespan. These are produced according to a keyframe trigger policy, which means it is not a fixed-rate output.

Additionally, it produces health metrics that help debug and assess the state of the module.

## Functionality

On a high level, this package provides functions for:

- feature detection on left camera images
- perform temporal tracking in the left camera with quality gates
- perform stereo matching between the left and right images with quality gates
- rolling buffer for track storage and PnP ready correspondences
- keyframe decision policy
- IMU buffering and preintegration
- packaging finalized keyframes

It does not own global localization or graph optimization.

## Public API

The main public class is [`VisualInertial`](include/visual_inertial_frontend/odometry.hpp), defined in `odometry.hpp`.

Typical usage looks like:

```cpp
#include "visual_inertial_frontend/odometry.hpp"

VisualInertial vio; // instance the VisualInertial class once
vio.setCalibration(camera_rig); // set the camera + IMU rig calibration once

vio.processImu(sample); // upon every new IMU sample, process it
FrameResult result = vio.processStereo(left_gray, right_gray, stamp); // on every new frame pair, process them and get a motion estimate

KeyframeEvent keyframe;
if (vio.tryPopFinalizedKeyframe(keyframe)) {
  // publish or forward to the backend
}
```

`processStereo(...)` returns a [`FrameResult`](include/visual_inertial_frontend/types.hpp) with the current local VO estimate, tracking state, and keyframe trigger decision. Finalized keyframes are produced asynchronously through the internal pending and ready queues and are retrieved with `tryPopFinalizedKeyframe(...)`.

## Processing flow

At a high level, the package runs in two parallel streams of work.

The first is frame rate visual processing:

1. `processStereo(...)` receives the new rectified stereo pair.
2. Existing left image tracks are propagated temporally.
3. Right image correspondences are refreshed through stereo tracking.
4. The track buffer is gated according to the params. We want to avoid spurious tracks from getting into our tracking.
5. If enough anchored correspondences exist, the frontend estimates a local visual odometry update.
6. Frontend health metrics are updated from the current tracking and PnP state.
7. The keyframe policy decides whether the current frame should become a keyframe.
8. If triggered, a pending keyframe event is queued.

The second is keyframe finalization:

1. IMU samples are continuously fed through `processImu(...)` into the preintegrator buffer.
2. A pending keyframe waits until there is enough IMU coverage for its interval.
3. `tryFinalizeOne()` builds the preintegrated IMU packet for that interval.
4. The completed `KeyframeEvent` is moved to the ready queue.
5. The caller retrieves it with `tryPopFinalizedKeyframe(...)`.

This split is why the per frame pose estimate and the finalized keyframe output are related but not identical concepts.

## Important data structures

The most important structures in this package are:

- [`VisualInertial::Params`](include/visual_inertial_frontend/odometry.hpp): top level configuration for tracking, stereo gating, feature top up, PnP, keyframe policy, and IMU preintegration
- [`FrameResult`](include/visual_inertial_frontend/types.hpp): per frame output from `processStereo(...)`, including the local pose estimate, keyframe trigger flag, track set, and frontend health
- [`FrontendHealth`](include/visual_inertial_frontend/types.hpp): compact summary of tracking quality, stereo quality, and PnP quality for the current interval
- [`ImuSample`](include/visual_inertial_frontend/types.hpp): the frontend IMU input type used by `processImu(...)`
- [`KeyframePolicy::Decision`](include/visual_inertial_frontend/keyframe_policy.hpp): result of the keyframe trigger policy, including both the boolean decision and the reasons behind it
- [`ImuPreintegrator::BuildResult`](include/visual_inertial_frontend/preintegrator.hpp): result of attempting to build a preintegrated IMU packet for a keyframe interval

The package also relies heavily on [`KeyframeEvent`](../visual_inertial_common/include/visual_inertial_common/types.hpp), which is defined in `visual_inertial_common` and is the finalized output handed to the backend. This definition lives in a "common" package as it is used by the frontend, backend libraries, and users of the whole system.

## Parameters

The top level params are defined in [`VisualInertial::Params`](include/visual_inertial_frontend/odometry.hpp). The easiest way to read them is by functional group.

Tracking and stereo:

- `target_features`: target number of left image features to keep alive during normal operation
- `stereo_epi_eps_y`: maximum allowed vertical mismatch between left and right tracks in rectified stereo
- `stereo_disp_min`: minimum allowed stereo disparity before a match is rejected
- `stereo_disp_max`: maximum allowed stereo disparity before a match is rejected
- `fb_thr2`: squared forward backward LK error threshold used for temporal and stereo gating

Degraded tracking thresholds:

- `degraded_min_tracks`: minimum number of surviving tracks before the frame is considered weak
- `degraded_min_track_retention`: minimum fraction of tracks that must survive from the previous frame before the frame is considered weak

Track top up and masking:

- `T_BC`: body to camera transform used to convert the internal camera pose into the published body pose
- `kf_ready_queue_max`: maximum number of finalized keyframes held in the ready queue
- `kf_pending_queue_max`: maximum number of not yet finalized keyframes held in the pending queue
- `mask_scale`: coarse grid scale used when building the feature top up exclusion mask
- `topup_grid_scale`: grid scale used when choosing spatially separated new features
- `max_total_tracks`: hard cap on the total number of live tracks
- `topup_burst_factor`: multiplier used when deciding how aggressively to refill tracks
- `topup_detect_factor`: multiplier used when overdetecting feature candidates before spatial selection

PnP:

- `pnp_iterations_count`: RANSAC iteration budget for `solvePnPRansac`
- `pnp_reproj_error_px`: reprojection error threshold in pixels for PnP inlier selection
- `pnp_confidence`: target RANSAC confidence for PnP
- `pnp_refine_max_iters`: maximum LM refinement iterations after the PnP inlier set is chosen
- `pnp_refine_eps`: LM refinement termination epsilon

Keyframe policy:

- `kf_policy_cfg.min_kf_dt_s`: minimum time between keyframes unless an early quality based trigger is allowed
- `kf_policy_cfg.max_kf_dt_s`: maximum time between keyframes before a keyframe is forced
- `kf_policy_cfg.min_trans_m`: translation trigger since the last keyframe
- `kf_policy_cfg.min_rot_deg`: rotation trigger since the last keyframe
- `kf_policy_cfg.min_tracks`: minimum live track count before the policy treats the frame as weak
- `kf_policy_cfg.min_shared_tracks`: minimum number of tracks shared with the previous keyframe
- `kf_policy_cfg.min_shared_ratio`: minimum fraction of tracks shared with the previous keyframe
- `kf_policy_cfg.min_pnp_tracks`: minimum number of anchored tracks available for PnP
- `kf_policy_cfg.force_kf_on_max_interval`: whether the max interval trigger always forces a keyframe
- `kf_policy_cfg.allow_early_kf_on_quality_drop`: whether weak tracking can trigger a keyframe before the minimum interval

IMU preintegration:

- `imu_cfg.gyro_noise_density`: gyroscope continuous time noise density
- `imu_cfg.accel_noise_density`: accelerometer continuous time noise density
- `imu_cfg.gyro_bias_rw`: gyroscope bias random walk level
- `imu_cfg.accel_bias_rw`: accelerometer bias random walk level
- `imu_cfg.gravity_mps2`: gravity magnitude used by preintegration
- `imu_cfg.max_buffer_s`: maximum age of IMU samples kept in the buffer
- `imu_cfg.keep_anchor`: whether to keep the last sample at or before the consumed interval end as the next anchor

For the exact defaults and types, use the header definitions as the source of truth.

## Core headers

- `odometry.hpp`: main public API package entry point
- `types.hpp`: frontend outputs and IMU input types
- `keyframe_policy.hpp`: keyframe decision policy and reporting
- `preintegrator.hpp`: IMU buffering and preintegrated packet construction - mostly a data handling wrapper around GTSAM's preintegrator
- `tracks_buffer.hpp`: aligned track storage for left, right, and anchored 3D data
- `feature_tracker.hpp`: OpenCV GPU LK tracking wrapper
- `feature_detector.hpp`: OpenCV GPU feature top-up wrapper

## Main components

[`VisualInertial`](include/visual_inertial_frontend/odometry.hpp) is the public API that owns the full frontend state and drives the package.

[`TracksBuffer`](include/visual_inertial_frontend/tracks_buffer.hpp) stores aligned track ids, left and right image observations, and anchor frame 3D points so the rest of the frontend can operate on a consistent view of feature state.

[`FeatureTracker`](include/visual_inertial_frontend/feature_tracker.hpp) wraps the CUDA LK tracking path used for both temporal tracking in the left camera and stereo tracking between the left and right views.

[`FeatureDetector`](include/visual_inertial_frontend/feature_detector.hpp) handles feature top up when the current track set becomes too sparse.

[`KeyframePolicy`](include/visual_inertial_frontend/keyframe_policy.hpp) decides when a frame should become a keyframe based on elapsed time, motion since the last keyframe, and track quality.

[`ImuPreintegrator`](include/visual_inertial_frontend/preintegrator.hpp) buffers IMU samples and builds preintegrated packets for the intervals between finalized keyframes.

## Outputs

The package produces two main kinds of output.

Frame rate output:

- [`FrameResult`](include/visual_inertial_frontend/types.hpp)

This contains the current local VO estimate, the keyframe trigger decision for the current frame, the currently tracked left image features, and a `FrontendHealth` summary.

Keyframe rate output:

- [`KeyframeEvent`](../visual_inertial_common/include/visual_inertial_common/types.hpp)

This is the finalized backend facing type that contains the keyframe observations, tracking summaries, and preintegrated IMU packet for the corresponding interval.

Taken together, these outputs let downstream code use the frontend both as a continuous local motion estimator and as a producer of backend ready keyframe events.

## Dependencies

This package depends mainly on:

- OpenCV, including CUDA tracking and feature detection paths
- GTSAM for IMU preintegration support
- `visual_inertial_common` for shared data structures such as `CameraRig`, `KeyframeEvent`, `ImuBias`, and serialized IMU packets

In practice, the frontend assumes:

- rectified grayscale stereo input
- IMU timestamps in the same clock domain as keyframe timestamps
- calibration provided by the caller before normal operation

## Tests

The package has unit tests wired into `colcon test`:

- [`test_keyframe_policy.cpp`](test/test_keyframe_policy.cpp)
- [`test_tracks_buffer.cpp`](test/test_tracks_buffer.cpp)
- [`test_preintegrator.cpp`](test/test_preintegrator.cpp)

Run them with:

```bash
colcon test --base-paths src/myslam_monorepo/src --packages-select visual_inertial_frontend --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/visual_inertial_frontend
```

## Relationship To The Node Layer

This package is a library only. ROS message conversion, topic wiring, and node execution live in [`visual_inertial`](../visual_inertial), which uses `VisualInertial` as the frontend compute engine.
