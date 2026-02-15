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
        uint16_t target_features = 500;
        float stereo_epi_eps_y = 1.5;
        float stereo_disp_min = 3.0;
        float stereo_disp_max = 255.0;
        float fb_thr2 = 3.0 * 3.0;

    };

    VisualInertial()
        : VisualInertial(Params{})
    {
    }

    VisualInertial(const Params &p);

    

    FrameResult processStereo(const cv::Mat &gray8_left,
                              const cv::Mat &gray8_right, double stamp);

    void setCalibration(const CameraRig &calibration)
    {
        calibration_ = calibration;
    }

private:
    Params params_;
    bool first_frame_ = true;

    CameraRig calibration_;
    TracksBuffer tracks_buffer_;
    FeatureDetector feature_detector_;
    FeatureTracker tracker_temporal_;
    FeatureTracker tracker_spatial_;

    cv::cuda::Stream stream_{};
    cv::cuda::GpuMat d_mask_{}; // mask on device for feature top-up
    cv::cuda::GpuMat d_gray8_left_{};
    cv::cuda::GpuMat d_gray8_right_{};
    cv::cuda::GpuMat d_gray8_left_prev_{};
    
};
