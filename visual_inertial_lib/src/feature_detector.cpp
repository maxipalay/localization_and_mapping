#include "visual_inertial_lib/feature_detector.hpp"

void FeatureDetector::ensureDetector_()
{
  if (!det_)
  {
    det_ = cv::cuda::createGoodFeaturesToTrackDetector(
        CV_8UC1,
        params_.max_corners,
        params_.quality_level,
        params_.min_distance,
        params_.block_size,
        params_.use_harris,
        params_.harris_k);
  }
}

void FeatureDetector::detectDevice(const cv::cuda::GpuMat &d_gray8,
                                   const cv::cuda::GpuMat &d_mask,
                                   cv::cuda::GpuMat &points,
                                   const int max_pts)
{
  if (d_gray8.empty() || d_gray8.type() != CV_8UC1)
  {
    points.release();
    return;
  }
  ensureDetector_();

  det_->setMaxCorners(max_pts);
  det_->detect(d_gray8, points, d_mask, *stream_);
}

int FeatureDetector::detect(const cv::cuda::GpuMat &d_gray8,
                            const cv::cuda::GpuMat &d_mask,
                            std::vector<cv::Point2f> &points,
                            const int max_pts)
{
  if (d_gray8.empty() || d_gray8.type() != CV_8UC1)
  {
    points.clear();
    return 0;
  }
  ensureDetector_();

  det_->setMaxCorners(max_pts);
  det_->detect(d_gray8, d_pts_, d_mask, *stream_);

  cv::Mat h;
  d_pts_.download(h, *stream_); // CV_32FC2, 1xN or Nx1

  stream_->waitForCompletion(); // sync only when you need results

  if (h.empty())
  {
    points.clear();
    return 0;
  }

  cv::Mat flat = h.reshape(2, h.rows * h.cols);
  const int N = flat.rows;
  tmp_.resize(N);
  for (int i = 0; i < N; ++i)
  {
    auto v = flat.at<cv::Vec2f>(i, 0);
    tmp_[i] = {v[0], v[1]};
  }
  points = tmp_;
  return N;
}