# esdf_slice_map_publisher

Loads a ROS occupancy-map YAML (`map_server` style `YAML + PGM`) and publishes it
as:

- `nav_msgs/OccupancyGrid` for RViz or downstream planning tools
- `sensor_msgs/PointCloud2` with intensity equal to distance-to-obstacle, so RViz can
  render an NVIDIA-style rainbow ESDF slice view

This is meant to work directly with the ESDF slice export from
`offline_dense_map_fusion`.

## Usage

```bash
ros2 run esdf_slice_map_publisher esdf_slice_map_publisher_node \
  --ros-args \
  -p map_yaml_path:=/tmp/online_mapping_sessions/<session>/offline_dense_map_fusion/esdf_slice_z_1_350_occupancy.yaml
```

Useful parameters:

- `map_yaml_path` (required): path to the occupancy YAML
- `topic` (default: `/esdf_slice_map`)
- `pointcloud_topic` (default: `/esdf_slice_pointcloud`)
- `frame_id` (default: `map`)
- `slice_height_m` (default: parsed from filename if possible, else `0.0`)
- `publish_occupancy` (default: `true`)
- `publish_pointcloud` (default: `true`)
- `publish_once` (default: `true`)
- `pointcloud_stride` (default: `1`)
- `max_pointcloud_distance_m` (default: unlimited)
- `publish_rate_hz` (default: `1.0`)

The node publishes with transient-local QoS, so RViz should receive the map even
if it starts after the first publication.

For lightweight visualization, a good first pass is:

```bash
ros2 run esdf_slice_map_publisher esdf_slice_map_publisher_node \
  --ros-args \
  -p map_yaml_path:=/tmp/online_mapping_sessions/<session>/offline_dense_map_fusion/esdf_slice_z_1_350_occupancy.yaml \
  -p publish_once:=true \
  -p pointcloud_stride:=4 \
  -p max_pointcloud_distance_m:=2.0
```

## RViz

For the rainbow slice look:

1. Add a `PointCloud2` display
2. Set topic to `/esdf_slice_pointcloud`
3. Set `Color Transformer` to `Intensity`
4. Set `Channel Name` to `intensity`
5. Enable `Use rainbow`
6. Set `Style` to `Flat Squares`
