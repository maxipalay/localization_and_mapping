# online_mapping_logger

Standalone ROS 2 logger for online mapping sessions. It records keyframes, RGB/depth images,
AprilTag detections, TF-resolved body-to-tag observations, and optimizer output for offline use.

For the OAK pipeline used in this workspace, the dense-mapping image pair should be logged from:

- `rgb_image_topic: /oak/left/image_synced`
- `rgb_camera_info_topic: /oak/left/image_synced/camera_info`
- `depth_image_topic: /oak/depth`
- `depth_camera_info_topic: /oak/depth/camera_info`

That pair is aligned for dense fusion. `oak/left/image_rect` is not the correct color partner for
`oak/depth` in this setup.

## Build note

This package depends on the separately built `visual_inertial` package. Isolated builds fail until
`visual_inertial` is discoverable through `CMAKE_PREFIX_PATH`, for example by sourcing the workspace
that installed it before running `colcon build`.
