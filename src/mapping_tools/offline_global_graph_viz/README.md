# offline_global_graph_viz

RViz visualization node for `offline_global_graph` optimized outputs.

## What it publishes

- Static TF frames for optimized keyframes and optimized tags
- Marker array with:
  - keyframe path
  - keyframe position spheres
  - keyframe labels
  - tag cubes
  - tag labels

## Usage

```bash
ros2 run offline_global_graph_viz offline_global_graph_viz_node \
  --ros-args -p session_dir:=/tmp/online_mapping_sessions/session_20260321_135348
```

Important parameters:

- `session_dir`
- `world_frame_id` default: `map`
- `keyframe_prefix` default: `optimized_kf_`
- `tag_prefix` default: `optimized_tag_`
- `publish_tf` default: `true`

Marker topic:

- `/offline_global_graph/markers`
