# offline_global_graph

Offline pose-graph refinement for sessions recorded by `online_mapping_logger`.

## Current model

The package now implements only the simple path you have been testing:

- initialize body poses from the logged online optimizer poses
- add fixed-covariance `between` factors between consecutive keyframes
- add gated body-to-tag observation factors as loop-closure constraints
- anchor the solve either to a chosen tag prior or, if none is provided, to the first keyframe

Everything else from the earlier experiments has been removed from the main optimizer path.

## Usage

```bash
ros2 run offline_global_graph offline_global_graph_cli \
  --session-dir /tmp/online_mapping_sessions/session_20260321_102832
```

Optional tag anchor:

```bash
ros2 run offline_global_graph offline_global_graph_cli \
  --session-dir /tmp/online_mapping_sessions/session_20260321_102832 \
  --map-anchor-tag-priors /tmp/online_mapping_sessions/session_20260321_102832/tag_priors.yaml \
  --map-anchor-tag-id 1
```

Optional tag filtering:

- `--only-tag-id INTEGER`
- `--exclude-tag-id INTEGER`

## Inputs used

The optimizer consumes:

- `optimized_pose_wb` for each keyframe
- logged tag observations expressed as `body -> tag`
- an optional tag prior pose from `tag_priors.yaml` for the chosen anchor tag

## Outputs

The optimizer writes:

- `offline_global_graph/optimized_keyframes.csv`
- `offline_global_graph/optimized_tags.yaml`
- `offline_global_graph/optimization_summary.yaml`

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

## Compatibility note

Sessions that logged tag TF in a frame other than the session body frame are still skipped.
