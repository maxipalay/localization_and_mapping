# offline_global_graph

Offline global graph refinement for sessions recorded by `online_mapping_logger`.

## Default workflow

- Loads a logged mapping session from disk
- Initializes body poses from the logged online optimizer poses
- Adds between factors between consecutive keyframes
- Uses logged frontend interval-health to inflate or skip weak between factors
- Adds body-to-tag observation factors from logged tag TF for loop closure
- Adds pose priors from the logged online optimizer covariance
- Anchors the graph with a prior on the first keyframe
- Writes optimized keyframe poses, optimized tag poses, and an optimization summary

This is the only supported path in the main CLI.

The current default behavior is equivalent to the old explicit command:

```bash
./build/offline_global_graph/offline_global_graph_cli \
  --session-dir /tmp/online_mapping_sessions/session_20260419_215555/ \
  --use-optimizer-pose-priors \
  --optimizer-pose-prior-covariance-scale 10.0 \
  --anchor-translation-sigma 0.3 \
  --anchor-rotation-sigma 0.5 \
  --tag-observation-huber-k 1.345
```

## Usage

```bash
ros2 run offline_global_graph offline_global_graph_cli --session-dir /tmp/online_mapping_sessions/session_20260321_102832
```

The main CLI is intentionally lightweight. The only supported options are:

- `--session-dir PATH`
- `--output-dir PATH`

## What it does not do

- It does not load or use `tag_priors.yaml`
- It does not anchor on tags
- It does not expose alternate main-CLI modes for tag-only optimization
- It does not expose the old broad tuning surface in the main CLI

## Session inputs used by the optimizer

The refinement path consumes these logged fields when present:

- `optimized_pose_wb`
- `optimization.pose_wb_covariance`
- `between_pose_prev_curr_body`
- `interval_health`
- tag observations logged as `body -> tag`

## Debugging observations

To compare each logged `body -> tag` observation against what the optimized result predicts:

```bash
ros2 run offline_global_graph offline_global_graph_debug_cli \
  --session-dir /tmp/online_mapping_sessions/session_20260321_102832
```

Useful filters:

- `--kf-id INTEGER`
- `--tag-id INTEGER`
- `--only-tag-id INTEGER`
- `--exclude-tag-id INTEGER`
- `--max-rows INTEGER`
- `--sort ascending|descending`
- `--csv-output PATH`

The debug CLI reruns the optimizer for the session, then prints:

- the logged `body_T_tag`
- the predicted `body_T_tag = world_T_body^-1 * world_T_tag`
- the residual between them
- translation error in meters
- rotation error in degrees

## Compatibility note

Old sessions recorded before the logger fix may still contain camera-frame tag TF. Those
observations are skipped because v1 expects logged `body -> tag` observations.
