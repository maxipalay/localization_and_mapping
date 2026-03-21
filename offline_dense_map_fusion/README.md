# offline_dense_map_fusion

Offline dense map fusion from `online_mapping_logger` sessions and `offline_global_graph`
optimized poses.

## What v1 does

- Reads the logged RGB-D session
- Reads optimized body poses from `offline_global_graph/optimized_keyframes.csv`
- Reads camera intrinsics from `calibration/rgb_camera_info.yaml`
- Reads a user-supplied `body -> camera` extrinsic YAML
- Assumes the logged depth is aligned to the RGB/left optical camera
- Backprojects RGB-D into a globally fused colored point cloud
- Applies voxel filtering during fusion

This is a CPU fusion backend. It is the data-path step needed before swapping the integrator
backend to `nvblox` when that dependency is available locally.

## Usage

```bash
ros2 run offline_dense_map_fusion offline_dense_map_fusion_cli \
  --session-dir /tmp/online_mapping_sessions/session_20260321_135348 \
  --body-to-camera-extrinsics /home/max/develop/workspaces/myslam/body_to_oak_left_optical.yaml
```

Optional flags:

- `--output-dir PATH`
- `--voxel-size FLOAT`
- `--depth-scale FLOAT`
- `--min-depth FLOAT`
- `--max-depth FLOAT`
- `--pixel-stride INT`

## Extrinsics YAML

```yaml
body_frame_id: body
camera_frame_id: oak_left_optical
position: [0.0, 0.0, 0.0]
orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

The transform is interpreted as `body_T_camera`.

## Outputs

- `fused_cloud.ply`
- `camera_poses.csv`
- `fusion_summary.yaml`
