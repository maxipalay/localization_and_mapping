# online_mapping_logger

Standalone ROS 2 logger for online mapping sessions. It records keyframes, RGB/depth images,
AprilTag detections, TF-resolved body-to-tag observations, and optimizer output for offline use.

For the current RealSense splitter pipeline in this workspace, the live logger should use:

- `body_frame_id: camera0_imu_frame`
- `rgb_image_topic: /camera0/realsense_splitter_node/output/infra_1`
- `rgb_camera_info_topic: /camera0/infra1/camera_info`
- `depth_image_topic: /camera0/realsense_splitter_node/output/depth`
- `depth_camera_info_topic: /camera0/depth/camera_info`

The logger also writes `calibration/body_to_rgb_camera.yaml` and
`calibration/body_to_depth_camera.yaml` by resolving TF from `body_frame_id` to the camera-info
frame ids at runtime.

This uses the emitter-off left IR image as the mapping image stream and the splitter depth stream
for depth. That pair is stable for logging, but it is not guaranteed to be pixel-aligned.

## Build note

This package depends on the separately built `visual_inertial` package. Isolated builds fail until
`visual_inertial` is discoverable through `CMAKE_PREFIX_PATH`, for example by sourcing the workspace
that installed it before running `colcon build`.
