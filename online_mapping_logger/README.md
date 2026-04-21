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

## Compare Online vs Offline Optimizer Poses

The helper script `scripts/export_online_optimizer_poses.py` exports the raw online optimizer poses
from `keyframe_manifest.csv` into `offline_global_graph_viz` CSV format and compares them against
`offline_global_graph/optimized_keyframes.csv` when present.

Print a summary and export a sidecar visualization session:

```bash
python3 mapping_tools/online_mapping_logger/scripts/export_online_optimizer_poses.py \
  /tmp/online_mapping_sessions/session_20260419_215555 \
  --copy-offline-tags
```

Print per-keyframe online-vs-offline pose deltas:

```bash
python3 mapping_tools/online_mapping_logger/scripts/export_online_optimizer_poses.py \
  /tmp/online_mapping_sessions/session_20260419_215555 \
  --per-kf
```

Print only the worst per-keyframe deltas:

```bash
python3 mapping_tools/online_mapping_logger/scripts/export_online_optimizer_poses.py \
  /tmp/online_mapping_sessions/session_20260419_215555 \
  --per-kf --top-k 20
```

By default the script writes a sidecar session directory named
`<session_dir>_online_viz/offline_global_graph/optimized_keyframes.csv`. You can visualize the raw
online optimizer trajectory with:

```bash
ros2 run offline_global_graph_viz offline_global_graph_viz_node \
  --ros-args -p session_dir:=/tmp/online_mapping_sessions/session_20260419_215555_online_viz
```

And visualize the offline-optimized result from the original session with:

```bash
ros2 run offline_global_graph_viz offline_global_graph_viz_node \
  --ros-args -p session_dir:=/tmp/online_mapping_sessions/session_20260419_215555
```
