# offline_global_graph

Offline global graph builder and optimizer for sessions recorded by `online_mapping_logger`.

## What v1 does

- Loads a logged mapping session from disk
- Initializes body poses from the logged optimized body poses
- Adds between factors between consecutive keyframes
- Adds body-to-tag observation factors from logged tag TF
- Adds one strong anchor prior and optional soft tag priors
- Writes optimized keyframe poses, optimized tag poses, and an optimization summary

## Usage

```bash
ros2 run offline_global_graph offline_global_graph_cli --session-dir /tmp/online_mapping_sessions/session_20260321_102832
```

Optional flags:

- `--output-dir PATH`
- `--only-tag-id INTEGER`
- `--exclude-tag-id INTEGER`
- `--between-translation-sigma FLOAT`
- `--between-rotation-sigma FLOAT`
- `--tag-translation-sigma FLOAT`
- `--tag-rotation-sigma FLOAT`
- `--soft-prior-translation-sigma FLOAT`
- `--soft-prior-rotation-sigma FLOAT`
- `--anchor-translation-sigma FLOAT`
- `--anchor-rotation-sigma FLOAT`
- `--robust-huber-k FLOAT`

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

## Tag prior schema

`tag_priors.yaml` may contain either `tags:` or `priors:` as a sequence:

```yaml
tags:
  - id: 1
    anchor: true
    position: [0.0, 0.0, 0.0]
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
    translation_sigma_m: 0.01
    rotation_sigma_rad: 0.02
  - id: 2
    position: [1.0, 0.0, 0.0]
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
    translation_sigma_m: 0.10
    rotation_sigma_rad: 0.20
```

If no tag prior file is provided, the graph falls back to anchoring the first keyframe.

## Compatibility note

Old sessions recorded before the logger fix may still contain camera-frame tag TF. Those
observations are skipped because v1 expects logged `body -> tag` observations.
