# online_mapping_logger

Standalone ROS 2 logger for online mapping sessions. It records keyframes, RGB/depth images,
AprilTag detections, TF-resolved body-to-tag observations, and optimizer output for offline use.

## Build note

This package depends on the separately built `visual_inertial` package. Isolated builds fail until
`visual_inertial` is discoverable through `CMAKE_PREFIX_PATH`, for example by sourcing the workspace
that installed it before running `colcon build`.
