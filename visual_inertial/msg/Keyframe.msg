# Keyframe.msg
# A keyframe event emitted by the frontend at ~1–10 Hz.
# Arrays are aligned by index: i-th element across arrays belongs to the same track.

std_msgs/Header header          # header.stamp = keyframe time, header.frame_id = "world" (recommended)

uint64 kf_id

# Pose of LEFT camera in world coordinates (World <- Camera).
# i.e., p_world = T_WC * p_cam
geometry_msgs/Pose pose_wc

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
