# visual_inertial

## Overview

ROS 2 runtime package for the visual inertial stack.

This package is the ROS boundary for the rest of `visual_inertial_odometry`. It owns the executable nodes, the ROS messages they exchange, and the transport code that converts between ROS messages and the underlying C++ library types.

The estimator logic itself lives in:

- [`visual_inertial_frontend`](../visual_inertial_frontend)
- [`visual_inertial_localization`](../visual_inertial_localization)
- [`visual_inertial_optimization`](../visual_inertial_optimization)

This package is what exposes those libraries as ROS nodes. Launch files and the default runtime YAML now live in [`visual_inertial_bringup`](../visual_inertial_bringup).

## What this package owns

This package contains:

- the main runtime nodes
  - `visual_inertial` (executable: `tracking_node`)
  - `localization_node`
  - `optimization_node`
- helper visualization nodes
  - `tracks_viz_node`
  - `path_viz_node`
- the ROS message types exchanged between nodes
- node parameter handlers
- the transport layer under `src/transport` that converts between ROS messages and the shared or library types

## Runtime modes

Two main runtime modes are supported.

Mapping mode:

- `visual_inertial -> optimization_node` via `Keyframe`
- `optimization_node -> visual_inertial` via `ImuBias`

In this mode, the frontend publishes keyframes and the backend optimizes them without tag based global correction.

<p align="center">
  <img src="../visual_inertial_bringup/docs/rosgraph_mapping.png" alt="visual_inertial mapping mode ROS graph" width="920" style="border-radius: 12px;" />
</p>

Localization mode:

- `visual_inertial -> localization_node` via `Keyframe`
- `visual_inertial -> optimization_node` via `Keyframe`
- `localization_node -> optimization_node` via `LocalizationCommand`
- `optimization_node -> localization_node` via `LocalizationFeedback`
- `optimization_node -> visual_inertial` via `ImuBias`

In this mode, the localization node watches tag detections and keyframes, sends localization commands to the optimizer, and receives backend `map -> odom` feedback back from the optimizer.

<p align="center">
  <img src="../visual_inertial_bringup/docs/rosgraph_localization.png" alt="visual_inertial localization mode ROS graph" width="920" style="border-radius: 12px;" />
</p>

## Main nodes

### `visual_inertial` (`tracking_node`)

The `visual_inertial` node wraps [`visual_inertial_frontend`](../visual_inertial_frontend).

It mainly:

- subscribes to stereo images, camera info, and IMU
- feeds IMU bias updates back into the frontend
- publishes tracks and frontend health
- publishes finalized keyframes for the backend
- publishes the frontend odometry TF

### `localization_node`

`localization_node` wraps [`visual_inertial_localization`](../visual_inertial_localization).

It mainly:

- subscribes to keyframes
- subscribes to AprilTag detections
- uses TF to resolve detections into body frame observations
- publishes `LocalizationCommand`
- subscribes to `LocalizationFeedback`

### `optimization_node`

`optimization_node` wraps [`visual_inertial_optimization`](../visual_inertial_optimization).

It mainly:

- subscribes to keyframes
- optionally waits for localization commands in localization mode
- runs backend updates at keyframe rate
- publishes optimized state summaries
- publishes IMU bias feedback
- publishes localization feedback
- publishes the `map -> odom` TF

### Helper nodes

The package also includes lightweight helper nodes:

- `tracks_viz_node` for feature track overlays
- `path_viz_node` for a body path view from TF

These helper nodes are optional. Bringup decides whether they run.

## Messages and transport

This package defines the ROS seam between the runtime nodes. The messages under [`msg/`](msg) are the public ROS side of that seam.

The main ones are:

- `Keyframe.msg`
- `Tracks.msg`
- `FrontendHealth.msg`
- `FrontendIntervalHealth.msg`
- `OptimizationResult.msg`
- `OptimizationStats.msg`
- `OptimizedKeyframePose.msg`
- `ImuBias.msg`
- `LocalizationCommand.msg`
- `LocalizationFeedback.msg`
- `LocalizationPosePrior.msg`

The transport layer under [`src/transport`](src/transport) converts between these ROS messages and the internal C++ types used by the frontend, localization, optimization, and common packages.

That split is one of the main reasons this package exists. The libraries stay mostly ROS-free, and `visual_inertial` handles the message side.

## Configuration

This package owns the node side of the runtime configuration surface.

The main places to look are:

- node parameter headers under [`include/visual_inertial/`](include/visual_inertial)

Each node declares and owns its own runtime parameters. The default runtime YAML and the launch entry points live in [`visual_inertial_bringup`](../visual_inertial_bringup).

## Tests

Launch tests for the runtime graph now live in [`visual_inertial_bringup`](../visual_inertial_bringup).
