std_msgs/Header header   # stamp = keyframe time
uint64 kf_id

# Bias in body/IMU frame
geometry_msgs/Vector3 accel_bias   # m/s^2
geometry_msgs/Vector3 gyro_bias    # rad/s