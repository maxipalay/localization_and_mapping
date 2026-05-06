# ply_pointcloud_viz

Loads an ASCII mesh PLY, voxelizes its vertices, and publishes one colored point
per occupied voxel as `sensor_msgs/msg/PointCloud2`.

This is a lightweight visualization bridge for RViz and does not depend on
`nvblox`.

Example:

```bash
ros2 run ply_pointcloud_viz ply_pointcloud_viz_node \
  --ros-args \
  -p ply_path:=/tmp/online_mapping_sessions/session_20260423_212723/offline_dense_map_fusion/fused_mesh.ply \
  -p frame_id:=map \
  -p voxel_size_m:=0.05 \
  -p vertex_stride:=4
```

In RViz, add:

- `PointCloud2`

and point it to:

- `/ply_pointcloud_viz/points`

Recommended RViz settings:

- `Color Transformer`: `RGB8`
- `Style`: `Flat Squares`
- `Size (m)`: close to `voxel_size_m`
