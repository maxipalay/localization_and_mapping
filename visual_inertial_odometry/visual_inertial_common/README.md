# visual_inertial_common

## Overview

Shared types for the visual inertial stack.

This package is here so the frontend, localization, optimization, and ROS node layer can use the same core data without depending on each other directly. It is intentionally small. It mostly just defines the structs the rest of the stack already passes around.

## What this package owns

The main public header is:

- [`include/visual_inertial_common/types.hpp`](include/visual_inertial_common/types.hpp)

That header defines the shared types used between packages, including:

- `Camera` and `CameraRig`
- `FrontendIntervalHealth`
- `KeyframeEvent`
- `ImuBias`
- `ViState`
- `PreintegratedImuPacket`

### Most important types

- [`CameraRig`](include/visual_inertial_common/types.hpp): stereo calibration shared by the frontend and backend
- [`KeyframeEvent`](include/visual_inertial_common/types.hpp): the main handoff from the frontend to the backend
- [`FrontendIntervalHealth`](include/visual_inertial_common/types.hpp): quality summary for one frontend interval
- [`ImuBias`](include/visual_inertial_common/types.hpp): shared accel and gyro bias estimate
- [`PreintegratedImuPacket`](include/visual_inertial_common/types.hpp): serialized preintegrated IMU payload carried with keyframes