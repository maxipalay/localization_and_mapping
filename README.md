# TODO

- expose parameters on a single unified params file
- work on estimating camera<->IMU offset
- filter-based fast prediction

## Feature tracking and distribution
- Grid-based / cell-balanced top-up (prefer empty regions, newly visible FOV areas). You tried it; a bit slower; stashed as a potential upgrade.

- Track age / score / stability: prefer older tracks for PnP and keyframe overlap; drop flaky tracks sooner.

- Adaptive feature budget: target more features in low texture, fewer in high texture (keeps compute stable).


## Gating and measurement quality

- Better PnP health metrics: compute and log reprojection RMSE, inlier ratio, and failure reasons.

- Make PnP more deterministic: refine only when inliers are strong; adapt RANSAC iterations.


## Keyframes and frontend policy

- Keyframe policy tuned for parallax, not just time: avoid KF spam when stationary.

- Better chaining when KFs drop: tolerate gaps or enforce strictness + auto-reset.

## Backend optimization

- Robust noise models (Huber/Cauchy on stereo factors) and/or tune noise params.

- Residual-based pruning/downweighting of bad landmark observations.

## Landmark lifecycle rules:

- don’t trust/init until seen in multiple KFs (or sufficient parallax)

- Window tuning (KF count, lag) vs compute.

## IMU / timing robustness

- Clean up camera–IMU time offset estimation (remove “coverage margin” hacks).

- Optional gyro-only prediction as a PnP initial guess / degraded mode (low effort, moderate benefit).

## Reliability, recovery, diagnostics

- Degraded mode: when tracking/PnP is bad, stop updating pose, keep tracking, or reset gracefully.

- Backend exception recovery: catch smoother failures, reset window, continue from last good.

- Publish a “health” topic:

    - tracks alive, stereo-valid count, pnp inliers, rmse

## Visualization and debugging

- Other useful vizualization tools