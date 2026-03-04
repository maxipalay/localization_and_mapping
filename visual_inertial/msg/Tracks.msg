# Tracks.msg
# A message containing the left camera tracks for a frame
std_msgs/Header header          # header.stamp = keyframe time, header.frame_id = "world" (recommended)

float32[] u_l
float32[] v_l