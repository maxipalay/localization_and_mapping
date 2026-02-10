#pragma once
#include <opencv2/core.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <mutex>
#include <vector>

namespace vio
{
    class FeatureDetector
    {
    public:
        struct Params
        {
            int max_corners = 500;
            double quality_level = 1e-3;
            double min_distance = 3.0;
            int block_size = 3;
            bool use_harris = false;
            double harris_k = 0.04;
            bool operator==(const Params &o) const
            {
                return max_corners == o.max_corners &&
                       quality_level == o.quality_level &&
                       min_distance == o.min_distance &&
                       block_size == o.block_size &&
                       use_harris == o.use_harris &&
                       harris_k == o.harris_k;
            }
            bool operator!=(const Params &o) const { return !(*this == o); }
        };

        FeatureDetector(const Params &p, cv::cuda::Stream *s)
            : params_(p),
              stream_(s)
        {
        }

        explicit FeatureDetector(cv::cuda::Stream* s) : FeatureDetector(Params{}, s) {}

        const Params &params() const { return params_; }

        // schedules the detection to happen on the internal stream
        void detectDevice(const cv::cuda::GpuMat &d_gray8,
                    const cv::cuda::GpuMat &d_mask,
                    cv::cuda::GpuMat &points,
                    const int max_pts);
        
        // waits for detection to complete and processes result
        int detect(const cv::cuda::GpuMat &d_gray8,
            const cv::cuda::GpuMat &d_mask, 
            std::vector<cv::Point2f> &points,
            const int max_pts);

    private:
        void ensureDetector_();

        Params params_;
        cv::cuda::Stream* stream_;
        cv::Ptr<cv::cuda::CornersDetector> det_; // created from params_

        // detect CPU buffers
        cv::cuda::GpuMat d_pts_;                 // output buffer (Nx1 or 1xN, CV_32FC2)
        std::vector<cv::Point2f> tmp_;           // host scratch
    };
}