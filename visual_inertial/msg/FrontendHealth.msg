# Frontend health metrics from the latest stereo/VIO frontend update.
std_msgs/Header header

uint8 STATE_UNKNOWN=0
uint8 STATE_TRACKING=1
uint8 STATE_DEGRADED=2
uint8 STATE_LOST=3

int32 num_tracks
int32 num_stereo_tracks
int32 num_pnp_candidates
int32 num_pnp_inliers

float64 pnp_inlier_ratio
float64 pnp_reproj_rmse_px
float64 track_coverage

bool pose_update_valid
uint8 state
