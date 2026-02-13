#pragma once

#include "visual_inertial_lib/tracks_buffer.hpp"
#include "visual_inertial_lib/feature_tracker.hpp"
#include "visual_inertial_lib/feature_detector.hpp"
#include "visual_inertial_lib/types.hpp"

#include <opencv2/core/cuda.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

class VisualInertial
{
public:
    struct Params
    {
    };

FrameResult processStereo(const cv::Mat &gray8_left,
                                         const cv::Mat &gray8_right, double stamp);

    private :

    Params params_;
};