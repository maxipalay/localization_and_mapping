// TODO:
// instead of instantiating new vectors each time, use scratch buffer + clear()
// tracks can agglomerate in certain regions, would be good to implement a strategy for topup that uses some sort of grid
// do we want to keep tracks age? maybe the older a track is the more I can trust it?

// special case to handle
// First keyframe
// There is no “previous keyframe time”:
// publish KF with time_start = time_end and no IMU packet (or packet marked invalid)
// backend uses priors to bootstrap, and the first IMU factor starts at KF0→KF1

// double check time sources to use, if we grab from frames header or we use the stamp provided by the node

#include "visual_inertial_frontend/odometry.hpp"

// Inputs: pl[i], pr[i] in pixels (rectified); intrinsics fx, fy, cx, cy; baseline B (meters)
bool triangulateRectified(
    const cv::Point2f &pl, const cv::Point2f &pr,
    float fx, float fy, float cx, float cy, float B,
    cv::Point3f &X_out)
{
    const float d = pl.x - pr.x; // disparity

    const float z = fx * B / d;

    if (!std::isfinite(z) || z <= 0.f)
        return false;

    const float x = (pl.x - cx) * z / fx;
    const float y = (pl.y - cy) * z / fy;
    X_out = cv::Point3f(x, y, z);

    return std::isfinite(x) && std::isfinite(y);
}

VisualInertial::VisualInertial(const Params &p)
    : params_(p),
      feature_detector_(FeatureDetector::Params{}, &stream_),
      tracker_temporal_(&stream_),
      tracker_spatial_(&stream_),
      imu_preint_()
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

void VisualInertial::processImu(const ImuSample &sample)
{
    imu_preint_.push(sample.t_s, sample.accel, sample.gyro);
    return;
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

    ///
    /// SECTION - Temporal tracking (FW+BW pass)
    ///

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

    ///
    /// SECTION -  Spatial/stereo tracking
    ///

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

        keep_stereo[i] = (status_fw_stereo[i] != 0 && status_bw_stereo[i] != 0 && dx * dx + dy * dy < params_.fb_thr2 && dy_epi < params_.stereo_epi_eps_y && disp > params_.stereo_disp_min && disp <= params_.stereo_disp_max) ? 1 : 0;
        if (!keep_stereo[i])
            stereo_dropped++;
    }

    tracks_buffer_.setRightInPlace(pts_fw_stereo); // add right points information

    tracks_buffer_.applyKeepMask(keep_stereo); // stable compaction

    // std::cout << "temporal dropped: " << temporal_dropped << ", stereo dropped: " << stereo_dropped << std::endl;

    ///
    /// SECTION - PNP
    /// run pnp to get relative motion from previous to current frame

    // -----------------------------
    // 3) Build eligible PnP correspondences using TracksBuffer::gatherPnP()
    // -----------------------------
    std::vector<cv::Point3f> object_pts;
    std::vector<cv::Point2f> image_pts;
    std::vector<int> buf_idx; // map: eligible index -> buffer index (needed for B-style gating)

    tracks_buffer_.gatherPnP(object_pts, image_pts, &buf_idx);

    int pnp_corresp = (int)object_pts.size();
    if (object_pts.size() < 6)
        std::cout << "[PNP] not enough points!" << std::endl;

    if (object_pts.size() >= 6)
    {
        // -----------------------------
        // 4) solvePnPRansac (AP3P) + refineLM on inliers
        // -----------------------------
        cv::Mat rvec, tvec;
        std::vector<int> inliers;

        const int iterationsCount = 100;  // TODO tune
        const float reprojErrorPx = 2.0f; // TODO tune
        const double confidence = 0.999;

        cv::Matx33d K_left(
            calibration_.left.K(0, 0), calibration_.left.K(0, 1), calibration_.left.K(0, 2),
            calibration_.left.K(1, 0), calibration_.left.K(1, 1), calibration_.left.K(1, 2),
            calibration_.left.K(2, 0), calibration_.left.K(2, 1), calibration_.left.K(2, 2));

        bool ok = cv::solvePnPRansac(
            object_pts, image_pts, K_left, cv::noArray(),
            rvec, tvec,
            false,
            iterationsCount, reprojErrorPx, confidence,
            inliers,
            cv::SOLVEPNP_AP3P);

        if (!ok || inliers.size() < 6)
            std::cout << "[PNP] Error!" << std::endl;

        // Refine using LM on inliers
        {
            std::vector<cv::Point3f> obj_in;
            std::vector<cv::Point2f> img_in;
            obj_in.reserve(inliers.size());
            img_in.reserve(inliers.size());

            for (int j : inliers)
            {
                obj_in.push_back(object_pts[(size_t)j]);
                img_in.push_back(image_pts[(size_t)j]);
            }

            cv::TermCriteria crit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6); // TODO tune
            cv::solvePnPRefineLM(obj_in, img_in, K_left, cv::Mat(), rvec, tvec, crit);
        }

        auto pnp_inliers = (int)inliers.size();

        // -----------------------------
        // 5) Gate DB based on PnP inliers (B-style: drop eligible outliers only)
        // -----------------------------
        tracks_buffer_.gateByPnPInliers(buf_idx, inliers);

        // -----------------------------
        // 6) Convert (rvec,tvec) -> Eigen::Isometry3d T_Ck_Ck1
        // -----------------------------
        {
            cv::Mat R_cv;
            cv::Rodrigues(rvec, R_cv);

            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                    R(r, c) = R_cv.at<double>(r, c);
                t(r) = tvec.at<double>(r, 0);
            }

            auto T_Ck_Ck1 = Eigen::Isometry3d::Identity();
            T_Ck_Ck1.linear() = R;
            T_Ck_Ck1.translation() = t;

            vo_pose_abs_ = vo_pose_abs_ * T_Ck_Ck1.inverse();
            output.vo_pose_rel = T_Ck_Ck1.inverse();
        }
    }
    ///
    /// SECTION - TRIANGULATE AND UPDATE DB
    ///

    // re triangulate points in local frame and update DB
    const size_t N = tracks_buffer_.size();
    const auto &pl = tracks_buffer_.pl();
    const auto &pr = tracks_buffer_.pr();
    const auto &has_r = tracks_buffer_.hasRight();

    std::vector<cv::Point3f> X_curr;
    std::vector<uint8_t> valid_X;
    X_curr.resize(N);
    valid_X.resize(N);

    for (size_t i = 0; i < N; ++i)
    {
        if (!has_r[i])
        {
            valid_X[i] = 0;
            X_curr[i] = cv::Point3f(0.f, 0.f, 0.f);
            continue;
        }

        cv::Point3f X;
        const bool ok = triangulateRectified(
            pl[i], pr[i],
            calibration_.left.fx(), calibration_.left.fy(), calibration_.left.cx(), calibration_.left.cy(),
            calibration_.baseline,
            X);

        valid_X[i] = ok ? uint8_t{1} : uint8_t{0};
        X_curr[i] = ok ? X : cv::Point3f(0.f, 0.f, 0.f);
    }

    // Store as "prev 3D" for next frame's PnP
    tracks_buffer_.setPrev3DAll(X_curr, &valid_X);

    tracks_buffer_.applyKeepMask(valid_X); // drops those without valid 3D

    ///
    /// SECTION - TOPUP
    ///  top up features

    // build CPU mask around survivors
    cv::Mat cpu_mask = buildExclusionMask(d_gray8_left_.size(), tracks_buffer_.pl(), 6.0); // TODO: make exclusion radius a param
    d_mask_.upload(cpu_mask);

    std::vector<cv::Point2f> new_pts;
    int need_i = (int)params_.target_features - (int)tracks_buffer_.size();
    if (need_i < 0)
        need_i = 0;
    uint16_t need = (uint16_t)need_i;

    new_pts.reserve(need);
    // std::cout << "need new features: " << need << std::endl;
    feature_detector_.detect(d_gray8_left_, d_mask_, new_pts, need);

    tracks_buffer_.addNewLeft(new_pts);

    //
    // SECTION - KEYFRAME EVALUATION AND CREATION
    //
    // evaluate keyfrmae generation
    // if generate keyframe -> create a keyframe and put in return struct

    // 1) Ask the policy if we should create a keyframe
    KeyframePolicy::Input kf_in;
    kf_in.t_s = stamp;
    kf_in.T_WC = vo_pose_abs_;
    kf_in.pose_valid = true; // or: have_vo (if you only trust motion triggers when VO succeeded)

    kf_in.track_ids = tracks_buffer_.ids().data();
    kf_in.num_tracks = tracks_buffer_.ids().size();

    KeyframePolicy::Decision dec = keyframe_policy_.evaluate(kf_in);

    // 2) If yes: build the keyframe event (1–10 Hz, copying vectors is fine)
    if (dec.make_keyframe)
    {
        KeyframeEvent ev;
        ev.kf_id = next_kf_id_++;
        ev.t_start = timestamp_last_kf_;
        ev.t_end = stamp;

        const Eigen::Isometry3d T_BC = params_.T_BC; ///* your calibrated Body<-CamOptical (rot+trans) */;
        const Eigen::Isometry3d T_CB = T_BC.inverse();

        ev.T_WC = T_BC * vo_pose_abs_ * T_CB; // <-- STORE World<-Body in the KF (reuse field for now)
        // v.T_WC = vo_pose_abs_;

        // Tracks in deterministic order (same order across ids/pl/pr)
        ev.ids = tracks_buffer_.ids();
        ev.pl = tracks_buffer_.pl();
        ev.pr = tracks_buffer_.pr();
        ev.has_r = tracks_buffer_.hasRight(); // or whatever you name it

        // 4) Inform policy that the keyframe was accepted/created
        //    (policy stores last-kf pose/time + last-kf ids for overlap logic)
        keyframe_policy_.onKeyframeCreated(ev.kf_id, ev.t_end, vo_pose_abs_, ev.ids);
        output.kf_valid = true;

        // Right before buildAndConsume(...)
        std::cout << std::fixed << std::setprecision(6)
                  << "[KF/IMU] t0=" << ev.t_start
                  << " t1=" << ev.t_end
                  << " imu_size=" << imu_preint_.size()
                  << " imu_old=" << imu_preint_.oldestTime()
                  << " imu_new=" << imu_preint_.newestTime()
                  << " hasCoverage=" << (imu_preint_.hasCoverage(ev.t_end) ? 1 : 0)
                  << std::endl;

        if (!(ev.t_end > ev.t_start))
        {
            std::cout << "[KF/IMU] BAD INTERVAL: t0 >= t1" << std::endl;
        }

        ev.pim_bytes.clear();
        ev.has_imu = false;
        // Build + consume IMU preintegration for (t0, t1]
        // auto pkt_opt = imu_preint_.buildAndConsume(prev_kf_id_, ev.t_start, ev.kf_id, ev.t_end);

        // if (!pkt_opt || !pkt_opt->valid)
        // {
        //     std::cout << "aaa" << std::endl;
        //     // MVP behavior: still publish KF but mark IMU invalid / empty.
        //     // You can also choose to skip KF emission if you want.
        //     ev.pim_bytes.clear();
        //     ev.has_imu = false;
        // }
        // else
        // {
        //     ev.pim_bytes = pkt_opt->bytes; // or pkt_opt->pim_bytes depending on your struct
        //     ev.has_imu = true;
        // }

        output.kf = ev;

        timestamp_last_kf_ = stamp; // update last stamp
        prev_kf_id_ = ev.kf_id;
    }

    // debug

    if (dec.make_keyframe)
    {
        std::cout << keyframeDecisionDebugLine(dec) << "\n";
    }

    ///
    /// SECTION - DRAW RESULTS
    ///
    cv::Mat result_left_tracking_after_stereo;
    cv::cvtColor(gray8_left, result_left_tracking_after_stereo, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < tracks_buffer_.size(); i++)
    {
        circle(result_left_tracking_after_stereo, tracks_buffer_.pl()[i], 2, {0, 255, 0}, -1, cv::LINE_AA);
    }

    output.debug_viz = result_left_tracking_after_stereo;

    const Eigen::Isometry3d T_BC = params_.T_BC; ///* your calibrated Body<-CamOptical (rot+trans) */;
    const Eigen::Isometry3d T_CB = T_BC.inverse();

    output.vo_pose_abs = T_BC * vo_pose_abs_ * T_CB;

    ///
    /// SECTION - UPDATE BUFFERS
    ///

    d_gray8_left_prev_ = d_gray8_left_.clone();

    return output;
}