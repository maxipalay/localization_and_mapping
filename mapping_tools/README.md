# mapping_tools

`mapping_tools` holds the logging, offline optimization, fusion, and visualization tools used around the visual inertial stack.

This directory is not the online estimator. It is the rest of the workflow that starts once you want to record a session, clean it up offline, build a map, or inspect the result.

## What lives here

The main packages are:

- [`online_mapping_logger`](online_mapping_logger)
  - logs live mapping sessions from ROS topics
  - writes keyframes, images, detections, calibration, and optimizer output to disk
- [`offline_global_graph`](offline_global_graph)
  - refines the logged session offline as a pose graph
  - uses logged tag observations as loop-closure constraints
- [`offline_dense_map_fusion`](offline_dense_map_fusion)
  - fuses logged RGB-D data into a dense TSDF map and mesh

The rest are support tools:

- [`offline_global_graph_viz`](offline_global_graph_viz)
  - visualize offline graph results
- [`ply_pointcloud_viz`](ply_pointcloud_viz)
  - view exported point clouds or meshes
- [`esdf_slice_map_publisher`](esdf_slice_map_publisher)
  - publish 2D occupancy slices exported from dense fusion

## The main workflow

The normal path is:

1. run the live system and record a session with `online_mapping_logger`
2. solve the logged session offline with `offline_global_graph`
3. fuse the logged RGB-D data with `offline_dense_map_fusion`
4. inspect the result with the viz tools if needed

That is the core idea of this directory. The packages here are meant to chain together.

## Session flow

At a high level:

- `online_mapping_logger`
  - takes live ROS topics
  - writes a session directory
- `offline_global_graph`
  - takes that session directory
  - writes optimized keyframe poses and optimized tags
- `offline_dense_map_fusion`
  - takes the same session directory plus the optimized poses
  - writes a mesh, ESDF outputs, and fusion summaries

So the session directory is the handoff point between most of these tools.

## Relationship To The Rest Of The Repo

`mapping_tools` sits next to the online stack, not inside it.

The split is:

- [`../visual_inertial_odometry`](../visual_inertial_odometry)
  - online estimation
  - ROS runtime
  - localization and backend optimization
- `mapping_tools`
  - logging
  - offline cleanup
  - dense fusion
  - visualization
- [`../sensor_utils`](../sensor_utils)
  - sensor-specific runtime code

That keeps the online estimator separate from the offline mapping workflow.

## Where to start

If you want the main path, read these in order:

1. [`online_mapping_logger/README.md`](online_mapping_logger/README.md)
2. [`offline_global_graph/README.md`](offline_global_graph/README.md)
3. [`offline_dense_map_fusion/README.md`](offline_dense_map_fusion/README.md)

Then use the viz package docs as needed.
