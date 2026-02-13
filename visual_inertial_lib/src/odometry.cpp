#include "visual_inertial_lib/odometry.hpp"

FrameResult VisualInertial::processStereo(const cv::Mat &gray8_left,
                                          const cv::Mat &gray8_right, double stamp)
{
    FrameResult output;

    // get current tracks from buffer
    // track stereo (temporal(previous frame to current) + spatial (left to right)) + gating
    // update (gate) tracks buffer with stereo track result
    // run pnp to get relative motion from previous to current frame
    // update tracks buffer gating by pnp inliers (or better said, remove pnp outliers, but keep those points that werent eligible for pnp)
    // re triangulate points in local frame and update DB
    // top up features
    // evaluate keyfrmae generation
    // if generate keyframe -> create a keyframe and put in return struct
    // convert pnp relative pose to body frame
    // compose estimate of absolute motion

    return output;
}