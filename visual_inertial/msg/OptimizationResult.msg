std_msgs/Header header
uint64 kf_id
float64 t_s

# Optimized poses in map/world coordinates.
geometry_msgs/Pose pose_wc_opt
geometry_msgs/Pose pose_wb_opt

visual_inertial/OptimizationStats stats

# Optimized bias in body/IMU frame.
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gyro_bias
