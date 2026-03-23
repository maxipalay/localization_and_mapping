# offline_dense_map_fusion

Offline dense map fusion from `online_mapping_logger` sessions and `offline_global_graph`
optimized poses.

## What it does

- Reads the logged RGB-D session
- Reads optimized body poses from `offline_global_graph/optimized_keyframes.csv`
- Reads camera intrinsics and distortion from the logged mapping image stream camera info
  (typically `calibration/rgb_camera_info.yaml`)
- Reads a user-supplied `body -> camera` extrinsic YAML
- Assumes the logged depth is aligned to the logged mapping image stream
- For the OAK pipeline here, that aligned pair is `oak/left/image_synced` +
  `oak/depth`, not `oak/left/image_rect` + `oak/depth`
- If the logged camera info includes `plumb_bob` or `rational_polynomial`
  distortion coefficients, they are passed through to `nvblox::Camera`
- Fuses depth and color into an `nvblox` TSDF map
- Exports a colored mesh PLY generated from the TSDF

This backend requires a CUDA-capable GPU visible at runtime because `nvblox`
creates CUDA streams during mapper construction.

## Usage

```bash
ros2 run offline_dense_map_fusion offline_dense_map_fusion_cli \
  --session-dir /tmp/online_mapping_sessions/session_20260321_135348 \
  --body-to-camera-extrinsics /home/max/develop/workspaces/myslam/body_to_oak_left_optical.yaml
```

The session should have been logged with:

```yaml
rgb_image_topic: /oak/left/image_synced
rgb_camera_info_topic: /oak/left/image_synced/camera_info
depth_image_topic: /oak/depth
depth_camera_info_topic: /oak/depth/camera_info
```

Optional flags:

- `--output-dir PATH`
- `--voxel-size FLOAT`
- `--depth-scale FLOAT`
- `--min-depth FLOAT`
- `--max-depth FLOAT`
- `--pixel-stride INT`
- `--crop-border-px INT`
- `--truncation-distance-vox FLOAT`
- `--max-weight FLOAT`
- `--mesh-min-weight FLOAT`

## Extrinsics YAML

```yaml
body_frame_id: body
camera_frame_id: oak_left_optical
position: [0.0, 0.0, 0.0]
orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

The transform is interpreted as `body_T_camera`.

## Outputs

- `fused_mesh.ply`
- `camera_poses.csv`
- `fusion_summary.yaml`

## Useful tuning

If the mesh is noisy, a good first pass is:

```bash
ros2 run offline_dense_map_fusion offline_dense_map_fusion_cli \
  --session-dir /tmp/online_mapping_sessions/<session> \
  --body-to-camera-extrinsics /home/max/develop/workspaces/myslam/src/body_to_oak_left_optical.yaml \
  --voxel-size 0.05 \
  --pixel-stride 4 \
  --crop-border-px 10 \
  --min-depth 0.3 \
  --max-depth 3.0 \
  --truncation-distance-vox 6.0 \
  --max-weight 10.0 \
  --mesh-min-weight 0.5
```
