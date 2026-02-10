#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/cudaoptflow.hpp> // SparsePyrLK
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

namespace vio
{
    class FeatureTracker
    {
    public:
        struct Params
        {
            cv::Size win_size = {21, 21};
            int max_level = 3;          // pyramid levels
            int num_iters = 15;         // iterations per level
            double min_eig_thr = 1e-12; // reject low-texture patches
            bool use_initial = false;   // if you seed initial flow

            bool operator==(const Params &o) const
            {
                return win_size == o.win_size && max_level == o.max_level &&
                       num_iters == o.num_iters && min_eig_thr == o.min_eig_thr &&
                       use_initial == o.use_initial;
            }
            bool operator!=(const Params &o) const { return !(*this == o); }
        };

        FeatureTracker(const Params &p, cv::cuda::Stream *s)
            : params_(p),
              stream_(s)
        {
        }

        explicit FeatureTracker(cv::cuda::Stream *s) : FeatureTracker(Params{}, s) {}

        const Params &params() const { return params_; }

        // schedules the tracker on thew internal stream, non-blocking
        void trackDevice(const cv::cuda::GpuMat &d_gray_a,
                         const cv::cuda::GpuMat &d_gray_b,
                         const cv::cuda::GpuMat &d_pts_a,
                         cv::cuda::GpuMat &d_tracks_b,
                         cv::cuda::GpuMat &d_status);

        // waits for track to finish, blocking
        int track(const cv::cuda::GpuMat &d_prev_gray,
                  const cv::cuda::GpuMat &d_curr_gray,
                  const std::vector<cv::Point2f> &prev_pts,
                  std::vector<cv::Point2f> &curr_pts,
                  std::vector<unsigned char> &status);

        explicit FeatureTracker(const Params &p,
                                int capacity = 2000,
                                cv::cuda::Stream *stream = nullptr);

        void reserve(int capacity); // grow prealloc if needed

    private:
        void ensureTracker_();
        void ensureCapacity_(int need);

        Params params_;
        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> tracker_;
        cv::cuda::Stream *stream_;

        cv::cuda::GpuMat zero_mask{};
        int capacity_ = 0;
        // detect CPU buffers
        cv::cuda::GpuMat d_prev_pts_, d_curr_pts_, d_status_, d_err_;
        // pinned host buffers + views (1xCapacity)
        cv::cuda::HostMem h_prev_pts_host_, h_curr_pts_host_, h_status_host_;
        cv::Mat h_prev_pts_view_, h_curr_pts_view_, h_status_view_;
    };
}