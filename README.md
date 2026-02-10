# visual_inertial_lib

- feature_detector - encapsulation of feature detector
- feature_tracker - encapsulation of feature tracker
- tracks_buffer - a buffer that holds tracks frame-to-frame. Useful for pnp and keyframe generation
- keyframe_policy - the policy which defines when we should make a keyframe
- odometry - our main API, holds VisualInertial
- tracking_pipeline - logic related to visual features tracking
- imu_pipeline - logic related to handling of IMU data
- types - definition of any custom types

# visual_inertial_optimization

- keyframe_buffer - holds last K keyframes to be used in optimization
- optimizer - encapsulates the GTSAM optimizer
- odometry_optimization - our main API for optimization
