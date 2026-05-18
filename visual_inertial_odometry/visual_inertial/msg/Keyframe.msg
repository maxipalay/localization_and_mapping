# Keyframe.msg
# A keyframe event emitted by the frontend at ~1–10 Hz.
# Arrays are aligned by index: i-th element across arrays belongs to the same track.

std_msgs/Header header          # header.stamp = keyframe time, header.frame_id = "odom" (recommended)

float32 t_start
float32 t_end

uint64 kf_id

# Exported frontend pose in the odom/startup frame (Odom <- Body).
# i.e., p_odom = T_OB * p_body
geometry_msgs/Pose pose_odom_body

# Frontend VO between measurement from previous accepted keyframe body to current body.
# Convention matches GTSAM BetweenFactor(x_{k-1}, x_k, meas): meas = T_Bkm1_Bk.
geometry_msgs/Pose between_pose_prev_curr
uint8 has_vo_between

FrontendIntervalHealth interval_health

# Track observations (aligned arrays)
uint32[] track_ids

# Left pixel coordinates
float32[] u_l
float32[] v_l

# Right pixel coordinates (only valid where has_right[i] != 0)
float32[] u_r
float32[] v_r
uint8[]   has_right

# Optional: why the keyframe was created (bitmask from your policy, 0 if unused)
uint32 kf_reason_mask

# IMU preintegration payload (boost-serialized GTSAM PreintegratedCombinedMeasurements)
uint8[] pim_bytes
uint8 has_imu
