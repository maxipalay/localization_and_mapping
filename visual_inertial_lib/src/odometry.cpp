// TODO:
// instead of instantiating new vectors each time, use scratch buffer + clear()
// tracks can agglomerate in certain regions, would be good to implement a strategy for topup that uses some sort of grid
// do we want to keep tracks age? maybe the older a track is the more I can trust it?

#include "visual_inertial_lib/odometry.hpp"

// Inputs: pl[i], pr[i] in pixels (rectified); intrinsics fx, fy, cx, cy; baseline B (meters)
bool triangulateRectified(
    const cv::Point2f& pl, const cv::Point2f& pr,
    float fx, float fy, float cx, float cy, float B,
    cv::Point3f& X_out)
{
  const float d = pl.x - pr.x;              // disparity

  const float z = fx * B / d;

  if (!std::isfinite(z) || z <= 0.f) return false;

  const float x = (pl.x - cx) * z / fx;
  const float y = (pl.y - cy) * z / fy;
  X_out = cv::Point3f(x, y, z);

  return true;
}


VisualInertial::VisualInertial(const Params &p)
    : params_(p),
      feature_detector_(FeatureDetector::Params{}, &stream_),
      tracker_temporal_(&stream_),
      tracker_spatial_(&stream_)
{
}

static inline cv::Mat buildExclusionMask(const cv::Size &sz,
                                         const std::vector<cv::Point2f> &keep,
                                         int radius_px)
{
    cv::Mat mask(sz, CV_8U, cv::Scalar(255));
    if (radius_px <= 0 || keep.empty())
        return mask;
    for (const auto &p : keep)
    {
        // skip points outside (paranoia)
        if (p.x < 0 || p.y < 0 || p.x >= sz.width || p.y >= sz.height)
            continue;
        cv::circle(mask, p, radius_px, cv::Scalar(0), -1, cv::LINE_AA);
    }
    return mask;
}

FrameResult VisualInertial::processStereo(const cv::Mat &gray8_left,
                                          const cv::Mat &gray8_right, double stamp)
{
    FrameResult output;

    // check our calibration is valid, if not return
    if (!calibration_.valid())
    {
        // std::cout << "[VisualInertial] Got frames but calibration is invalid!" << std::endl;
        return output;
    }

    // Upload both frames to device
    d_gray8_left_.upload(gray8_left, stream_);
    d_gray8_right_.upload(gray8_right, stream_);

    // first frame only
    if (first_frame_)
    {
        std::vector<cv::Point2f> new_pts;
        new_pts.reserve(params_.target_features);
        feature_detector_.detect(d_gray8_left_, d_mask_, new_pts, params_.target_features);
        tracks_buffer_.addNewLeft(new_pts);
        first_frame_ = false;
        d_gray8_left_prev_ = d_gray8_left_.clone();
        return output;
    }
    tracks_buffer_.beginFrame();
    
    // get current tracks from buffer
    const auto &pl_prev = tracks_buffer_.pl();

    /// Temporal tracking (FW+BW pass)

    // temporal feature tracking previous frame -> current frame
    std::vector<cv::Point2f> pts_fw;
    std::vector<unsigned char> status_fw;
    tracker_temporal_.track(d_gray8_left_prev_, d_gray8_left_, pl_prev, pts_fw, status_fw);

    // temporal feature tracking current frame -> previous frame
    std::vector<cv::Point2f> pts_bw;
    std::vector<unsigned char> status_bw;
    tracker_temporal_.track(d_gray8_left_, d_gray8_left_prev_, pts_fw, pts_bw, status_bw);

    // generate status vector
    // checks whether the points tracked forward and then backward are within a distance from the original points
    // generates the status vector

    uint16_t temporal_dropped = 0;
    std::vector<unsigned char> status_fb; // the status after applying the gate - this aligns with status_fw & status_bw
    status_fb.resize(pl_prev.size());
    for (size_t i = 0; i < status_fw.size(); ++i)
    {
        float dx = pl_prev[i].x - pts_bw[i].x;
        float dy = pl_prev[i].y - pts_bw[i].y;
        status_fb[i] = (status_fw[i] != 0 && status_bw[i] != 0 && dx * dx + dy * dy < params_.fb_thr2) ? 1 : 0;
        if (!status_fb[i])
            temporal_dropped++;
    }

    // update db with current frame points
    tracks_buffer_.setLeftInPlace(pts_fw); // commit new left points (same ordering)

    // apply the gating
    tracks_buffer_.applyKeepMask(status_fb); // stable compaction

    /// Spatial/stereo tracking

    // spatial/cross camera feature tracking: left -> right  + backward pass
    std::vector<cv::Point2f> pts_fw_stereo;
    std::vector<unsigned char> status_fw_stereo;
    tracker_spatial_.track(d_gray8_left_, d_gray8_right_, tracks_buffer_.pl(), pts_fw_stereo, status_fw_stereo);

    std::vector<cv::Point2f> pts_bw_stereo;
    std::vector<unsigned char> status_bw_stereo;
    tracker_spatial_.track(d_gray8_right_, d_gray8_left_, pts_fw_stereo, pts_bw_stereo, status_bw_stereo);
    // std::cout << tracks_buffer_.pl().size() << " " << pts_bw_stereo.size() << std::endl;
    // gating/update db
    uint16_t stereo_dropped = 0;
    std::vector<unsigned char> keep_stereo;
    keep_stereo.resize(tracks_buffer_.pl().size());
    for (size_t i = 0; i < status_fw_stereo.size(); ++i)
    {
        float dx = tracks_buffer_.pl()[i].x - pts_bw_stereo[i].x;
        float dy = tracks_buffer_.pl()[i].y - pts_bw_stereo[i].y;

        float dy_epi = std::fabs(pts_fw_stereo[i].y - tracks_buffer_.pl()[i].y); // epipolar line ~ horizontal
        float disp = tracks_buffer_.pl()[i].x - pts_fw_stereo[i].x;              // positive disparity expected

        keep_stereo[i] = (status_fw[i] != 0 && status_bw[i] != 0 && dx * dx + dy * dy < params_.fb_thr2 && dy_epi < params_.stereo_epi_eps_y && disp > params_.stereo_disp_min && disp <= params_.stereo_disp_max) ? 1 : 0;
        if (!keep_stereo[i])
            stereo_dropped++;
    }

    tracks_buffer_.setRightInPlace(pts_fw_stereo); // add right points information

    tracks_buffer_.applyKeepMask(keep_stereo); // stable compaction
    // std::cout << "temporal dropped: " << temporal_dropped << ", stereo dropped: " << stereo_dropped << std::endl;
    // run pnp to get relative motion from previous to current frame
    // update tracks buffer gating by pnp inliers (or better said, remove pnp outliers, but keep those points that werent eligible for pnp)
    // re triangulate points in local frame and update DB





    // top up features

    // build CPU mask around survivors
    cv::Mat cpu_mask = buildExclusionMask(d_gray8_left_.size(), tracks_buffer_.pl(), 3.0); // TODO: make exclusion radius a param
    d_mask_.upload(cpu_mask);

    std::vector<cv::Point2f> new_pts;
    uint16_t need = params_.target_features - tracks_buffer_.size();
    if (need < 0)
        need = 0;

    new_pts.reserve(need);
    // std::cout << "need new features: " << need << std::endl;
    feature_detector_.detect(d_gray8_left_, d_mask_, new_pts, need);

    tracks_buffer_.addNewLeft(new_pts);

    // evaluate keyfrmae generation
    // if generate keyframe -> create a keyframe and put in return struct
    // convert pnp relative pose to body frame
    // compose estimate of absolute motion

    cv::Mat result_left_tracking_after_stereo;
    cv::cvtColor(gray8_left, result_left_tracking_after_stereo, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < tracks_buffer_.size(); i++)
    {
        circle(result_left_tracking_after_stereo, tracks_buffer_.pl()[i], 2, {0, 255, 0}, -1, cv::LINE_AA);
    }

    output.debug_viz = result_left_tracking_after_stereo;
    
    d_gray8_left_prev_ = d_gray8_left_.clone();

    return output;
}