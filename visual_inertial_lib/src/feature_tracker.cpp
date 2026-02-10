#include "visual_inertial_lib/feature_tracker.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp> // CPU matchTemplate if desired
#include <cstring>           // std::memcpy

namespace vio
{

    FeatureTracker::FeatureTracker(const Params &p, int capacity, cv::cuda::Stream *stream)
        : params_(p), stream_(stream)
    {
        reserve(capacity);
    }

    void FeatureTracker::reserve(int capacity)
    {
        ensureCapacity_(capacity);
    }

    void FeatureTracker::ensureCapacity_(int need)
    {
        if (need <= capacity_)
            return;
        capacity_ = std::max(need, capacity_ > 0 ? capacity_ + capacity_ / 2 : need);

        // Device mats: 1 x capacity
        d_prev_pts_.create(1, capacity_, CV_32FC2);
        d_curr_pts_.create(1, capacity_, CV_32FC2);
        d_status_.create(1, capacity_, CV_8U);
        d_err_.create(1, capacity_, CV_32F); // LK "err" output if needed

        // Pinned host buffers (PAGE_LOCKED)
        h_prev_pts_host_ = cv::cuda::HostMem(cv::Size(capacity_, 1), CV_32FC2, cv::cuda::HostMem::PAGE_LOCKED);
        h_curr_pts_host_ = cv::cuda::HostMem(cv::Size(capacity_, 1), CV_32FC2, cv::cuda::HostMem::PAGE_LOCKED);
        h_status_host_ = cv::cuda::HostMem(cv::Size(capacity_, 1), CV_8U, cv::cuda::HostMem::PAGE_LOCKED);

        // Mat views to pinned memory
        h_prev_pts_view_ = h_prev_pts_host_.createMatHeader();
        h_curr_pts_view_ = h_curr_pts_host_.createMatHeader();
        h_status_view_ = h_status_host_.createMatHeader();
    }

    int FeatureTracker::track(const cv::cuda::GpuMat &d_prev_gray,
                              const cv::cuda::GpuMat &d_curr_gray,
                              const std::vector<cv::Point2f> &prev_pts,
                              std::vector<cv::Point2f> &curr_pts,
                              std::vector<unsigned char> &status)
    {
        curr_pts.clear();
        status.clear();

        const int K = static_cast<int>(prev_pts.size());
        if (K == 0)
            return 0;

        if (d_prev_gray.empty() || d_curr_gray.empty() ||
            d_prev_gray.type() != CV_8UC1 || d_curr_gray.type() != CV_8UC1)
        {
            return 0;
        }

        ensureTracker_();
        ensureCapacity_(K);

        // Fill pinned host upload buffer (1xK, CV_32FC2)
        {
            cv::Mat h_prev_roi = h_prev_pts_view_.colRange(0, K);
            // prev_pts memory layout matches CV_32FC2
            std::memcpy(h_prev_roi.ptr(), prev_pts.data(), K * sizeof(cv::Point2f));
        }

        // ROIs for device & host
        cv::cuda::GpuMat d_prev_roi = d_prev_pts_.colRange(0, K);
        cv::cuda::GpuMat d_curr_roi = d_curr_pts_.colRange(0, K);
        cv::cuda::GpuMat d_status_roi = d_status_.colRange(0, K);
        cv::cuda::GpuMat d_err_roi = d_err_.colRange(0, K);

        cv::Mat h_prev_roi = h_prev_pts_view_.colRange(0, K);
        cv::Mat h_curr_roi = h_curr_pts_view_.colRange(0, K);
        cv::Mat h_status_roi = h_status_view_.colRange(0, K);

        // Upload previous points (async)
        d_prev_roi.upload(h_prev_roi, *stream_);

        // LK (async)
        tracker_->calc(d_prev_gray, d_curr_gray,
                       d_prev_roi, d_curr_roi, d_status_roi, d_err_roi, *stream_);

        // Queue downloads (async)
        d_curr_roi.download(h_curr_roi, *stream_);
        d_status_roi.download(h_status_roi, *stream_);

        // Single sync for this call
        stream_->waitForCompletion();

        // Build outputs
        curr_pts.resize(K);
        const cv::Vec2f *pv = h_curr_roi.ptr<cv::Vec2f>(0);
        for (int i = 0; i < K; ++i)
        {
            curr_pts[i].x = pv[i][0];
            curr_pts[i].y = pv[i][1];
        }

        status.resize(K);
        std::memcpy(status.data(), h_status_roi.ptr(), K * sizeof(unsigned char));

        int n_valid = 0;
        for (int i = 0; i < K; ++i)
            n_valid += (status[i] != 0);
        return n_valid;
    }

    void FeatureTracker::ensureTracker_()
    {
        if (!tracker_)
        {
            tracker_ = cv::cuda::SparsePyrLKOpticalFlow::create(
                params_.win_size, params_.max_level, params_.num_iters);
            tracker_->setWinSize(params_.win_size);
            tracker_->setMaxLevel(params_.max_level);
            tracker_->setNumIters(params_.num_iters);
            tracker_->setUseInitialFlow(params_.use_initial);
        }
    }

    void FeatureTracker::trackDevice(const cv::cuda::GpuMat &d_gray_a,
                                     const cv::cuda::GpuMat &d_gray_b,
                                     const cv::cuda::GpuMat &d_pts_a,
                                     cv::cuda::GpuMat &d_tracks_b,
                                     cv::cuda::GpuMat &d_status)
    {
        if (d_gray_a.empty() || d_gray_b.empty() ||
            d_gray_a.type() != CV_8UC1 || d_gray_b.type() != CV_8UC1 ||
            d_pts_a.empty() || d_pts_a.type() != CV_32FC2)
        {
            d_tracks_b.release();
            d_status.release();
            return;
        }

        ensureTracker_();
        tracker_->calc(d_gray_a, d_gray_b, d_pts_a, d_tracks_b, d_status, zero_mask, *stream_);
    }

    // int FeatureTracker::track(const cv::cuda::GpuMat &d_prev_gray,
    //                           const cv::cuda::GpuMat &d_curr_gray,
    //                           const std::vector<cv::Point2f> &prev_pts,
    //                           std::vector<cv::Point2f> &curr_pts,
    //                           std::vector<unsigned char> &status)
    // {
    //     curr_pts.clear();
    //     status.clear();
    //     if (prev_pts.empty())
    //         return 0;

    //     // Make 1xN CV_32FC2 (what CUDA LK expects)
    //     cv::Mat h_prev(prev_pts);                  // Nx1 CV_32FC2
    //     cv::Mat h_prev_1xN = h_prev.reshape(2, 1); // 1xN CV_32FC2

    //     d_prev_pts_.upload(h_prev_1xN, *stream_);

    //     if (d_prev_gray.empty() || d_curr_gray.empty() ||
    //         d_prev_gray.type() != CV_8UC1 || d_curr_gray.type() != CV_8UC1 ||
    //         d_prev_pts_.empty() || d_prev_pts_.type() != CV_32FC2)
    //     {
    //         d_curr_pts_.release();
    //         d_status_.release();
    //         return 0;
    //     }

    //     ensureTracker_();

    //     tracker_->calc(d_prev_gray, d_curr_gray, d_prev_pts_, d_curr_pts_, d_status_, zero_mask, *stream_);

    //     cv::Mat h_curr, h_stat;
    //     d_curr_pts_.download(h_curr, *stream_);
    //     d_status_.download(h_stat, *stream_);

    //     stream_->waitForCompletion();

    //     // flatten back to vector<Point2f>

    //     // make it Nx1 to iterate naturally (or iterate cols)
    //     cv::Mat flat = h_curr.reshape(2, h_curr.rows * h_curr.cols); // Nx1 CV_32FC2
    //     curr_pts.resize(flat.rows);
    //     for (int i = 0; i < flat.rows; ++i)
    //     {
    //         auto v = flat.at<cv::Vec2f>(i, 0);
    //         curr_pts[i] = {v[0], v[1]};
    //     }
    //     status.assign(h_stat.datastart, h_stat.dataend);

    //     // Count valids

    //     int n_valid = 0;
    //     for (int i = 0; i < h_stat.rows * h_stat.cols; ++i)
    //         if (h_stat.data[i])
    //             ++n_valid;

    //     return n_valid;
    // }

}