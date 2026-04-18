std_msgs/Header header
uint64 kf_id
float64 t_s

# Optimized poses in map/world coordinates.
geometry_msgs/Pose pose_wc_opt
geometry_msgs/Pose pose_wb_opt

visual_inertial/OptimizationStats stats

# Optimized body-frame linear velocity in world coordinates.
bool has_velocity
geometry_msgs/Vector3 velocity_opt

# Optimized bias in body/IMU frame.
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gyro_bias

# Marginal covariance blocks from the fixed-lag smoother.
# pose_wb_covariance uses the Pose3 tangent ordering [rot_xyz, trans_xyz].
bool has_pose_wb_covariance
float64[36] pose_wb_covariance

# velocity_covariance is row-major xyz.
bool has_velocity_covariance
float64[9] velocity_covariance

# bias_covariance uses [accel_xyz, gyro_xyz].
bool has_bias_covariance
float64[36] bias_covariance
