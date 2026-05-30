# sensor_utils

`sensor_utils` holds sensor specific code.

This is where camera or hardware glue goes when it is needed by the rest of the repo but is not really part of the estimator or the mapping tools.

Right now this directory contains:

- [`realsense_utils`](realsense_utils)
  - RealSense bringup
  - emitter-state splitter node
  - IR gain correction nodes

The rough split is:

- `visual_inertial_odometry`
  - estimator code, ROS runtime nodes, bringup
- `mapping_tools`
  - logging, offline processing, visualization tools
- `sensor_utils`
  - sensor-specific support code

So this directory is the place for code that helps the system run on a particular sensor, but is not itself the main system.
