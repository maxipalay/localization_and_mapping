# visual_inertial_odometry

`visual_inertial_odometry` is the online state estimation stack used in this repo.

At a high level, it takes stereo images and IMU data, tracks motion locally, optionally uses AprilTags to tie that motion back to a map, and runs a fixed-lag backend to keep the state estimate consistent over time.

This directory holds the main estimator packages:

- [`visual_inertial_frontend`](visual_inertial_frontend)
  - stereo tracking
  - IMU ingest and preintegration
  - keyframe generation
  - frontend health
- [`visual_inertial_localization`](visual_inertial_localization)
  - AprilTag-based global correction logic
  - bootstrap and relocalization decisions
  - pose priors and overrides for the backend
- [`visual_inertial_optimization`](visual_inertial_optimization)
  - fixed-lag backend optimization
  - landmark management
  - optimized pose, velocity, and bias estimates
- [`visual_inertial_common`](visual_inertial_common)
  - shared types passed across the stack
- [`visual_inertial`](visual_inertial)
  - ROS nodes
  - ROS messages
  - transport code between ROS and the C++ libraries
- [`visual_inertial_bringup`](visual_inertial_bringup)
  - launch files
  - default runtime YAML
  - launch tests

## Stack components

The code is split by responsibility.

The frontend owns frame-rate work:

- track features
- estimate local motion
- decide when to make a keyframe
- package the data the backend needs

The localization package owns map-based correction:

- ingest tag detections
- decide when there is enough evidence to bootstrap or relocalize
- hand the backend soft priors or hard overrides

The optimization package owns the backend state:

- poses
- velocities
- IMU biases
- active landmarks

The ROS package wraps those libraries as nodes. The bringup package wires them into a running system for this repo.

## Runtime at a glance

The main runtime path is:

- stereo images and IMU go into `visual_inertial`
- `visual_inertial` publishes keyframes
- `visual_inertial_optimization` consumes those keyframes and updates the backend state
- `visual_inertial_optimization` feeds bias estimates back to `visual_inertial`

In localization mode there is one more loop:

- `visual_inertial_localization` watches keyframes and tag detections
- it sends localization commands to `visual_inertial_optimization`
- the optimizer sends localization feedback back

Bringup for this stack lives in [`visual_inertial_bringup`](visual_inertial_bringup). Sensor-specific camera glue lives outside this directory under [`../sensor_utils`](../sensor_utils).

## Where to start

If you just want to understand the stack, read in this order:

1. [`visual_inertial_bringup/README.md`](visual_inertial_bringup/README.md)
2. [`visual_inertial/README.md`](visual_inertial/README.md)
3. the package README for the part you care about
   - [`visual_inertial_frontend/README.md`](visual_inertial_frontend/README.md)
   - [`visual_inertial_localization/README.md`](visual_inertial_localization/README.md)
   - [`visual_inertial_optimization/README.md`](visual_inertial_optimization/README.md)
   - [`visual_inertial_common/README.md`](visual_inertial_common/README.md)

If you want the camera side too, read:

- [`../sensor_utils/realsense_utils/README.md`](../sensor_utils/realsense_utils/README.md)

## Tests

Coverage is split across the packages:

- `visual_inertial_frontend`
  - unit tests for keyframe policy, track buffering, and IMU preintegration
- `visual_inertial_localization`
  - unit tests for tag ingest, stability checks, and controller decisions
- `visual_inertial_optimization`
  - unit tests for backend initialization and health-based gating
- `visual_inertial_bringup`
  - launch tests for mapping mode and localization mode node graph wiring

Run the main subsystem test pass with:

```bash
colcon test --base-paths . --packages-select \
  visual_inertial \
  visual_inertial_bringup \
  visual_inertial_common \
  visual_inertial_frontend \
  visual_inertial_localization \
  visual_inertial_optimization \
  --event-handlers console_direct+
```

Then inspect results with:

```bash
colcon test-result --verbose
```

## Related parts of the repo

This stack is not the whole repo.

Other nearby pieces are:

- [`../sensor_utils`](../sensor_utils)
  - camera-specific runtime code
- [`../mapping_tools`](../mapping_tools)
  - session logging
  - offline graph work
  - dense fusion
  - visualization tools
