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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>

static inline void buildExclusionMaskDownsampledCPU(
    const cv::Size &full_sz,
    const std::vector<cv::Point2f> &keep,
    int scale,
    cv::Mat &mask_small,
    cv::Mat &mask_full)
{
    const int sw = (full_sz.width + scale - 1) / scale;
    const int sh = (full_sz.height + scale - 1) / scale;

    // allocate if needed
    if (mask_small.empty() || mask_small.cols != sw || mask_small.rows != sh)
        mask_small.create(sh, sw, CV_8U);
    if (mask_full.empty() || mask_full.size() != full_sz)
        mask_full.create(full_sz, CV_8U);

    // start with all allowed (255) in small mask
    mask_small.setTo(255);

    // mark occupied cells as forbidden (0)
    for (const auto &p : keep)
    {
        const int x = (int)p.x;
        const int y = (int)p.y;
        if ((unsigned)x >= (unsigned)full_sz.width || (unsigned)y >= (unsigned)full_sz.height)
            continue;

        const int sx = x / scale;
        const int sy = y / scale;
        if ((unsigned)sx < (unsigned)sw && (unsigned)sy < (unsigned)sh)
            mask_small.at<uint8_t>(sy, sx) = 0;
    }

    // upsample to full res with nearest neighbor (keeps 0/255 values)
    cv::resize(mask_small, mask_full, full_sz, 0, 0, cv::INTER_NEAREST);
}

static inline size_t countSetFlags(const std::vector<uint8_t> &flags)
{
    return static_cast<size_t>(std::count_if(
        flags.begin(), flags.end(),
        [](uint8_t v)
        { return v != 0; }));
}

static inline void selectDistributedTopupPoints(
    const cv::Size &full_sz,
    const std::vector<cv::Point2f> &candidates,
    int grid_scale,
    size_t max_pts,
    std::vector<cv::Point2f> &selected)
{
    selected.clear();
    if (candidates.empty() || max_pts == 0)
        return;

    const int scale = std::max(1, grid_scale);
    const int sw = (full_sz.width + scale - 1) / scale;
    const int sh = (full_sz.height + scale - 1) / scale;

    cv::Mat occupied(sh, sw, CV_8U, cv::Scalar(0));
    selected.reserve(std::min(max_pts, candidates.size()));

    for (const auto &p : candidates)
    {
        const int x = static_cast<int>(p.x);
        const int y = static_cast<int>(p.y);
        if ((unsigned)x >= (unsigned)full_sz.width || (unsigned)y >= (unsigned)full_sz.height)
            continue;

        const int sx = x / scale;
        const int sy = y / scale;
        if ((unsigned)sx >= (unsigned)sw || (unsigned)sy >= (unsigned)sh)
            continue;

        if (occupied.at<uint8_t>(sy, sx) != 0)
            continue;

        occupied.at<uint8_t>(sy, sx) = 1;
        selected.push_back(p);
        if (selected.size() >= max_pts)
            break;
    }
}

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

static inline void isometryToRvecTvec(
    const Eigen::Isometry3d &T,
    cv::Mat &rvec,
    cv::Mat &tvec)
{
    cv::Matx33d R_cv;
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
            R_cv(r, c) = T.linear()(r, c);
    }

    cv::Rodrigues(R_cv, rvec);
    tvec.create(3, 1, CV_64F);
    for (int r = 0; r < 3; ++r)
        tvec.at<double>(r, 0) = T.translation()(r);
}

VisualInertial::VisualInertial(const Params &p)
    : params_(p),
      feature_detector_(FeatureDetector::Params{}, &stream_),
      tracker_temporal_(&stream_),
      tracker_spatial_(&stream_),
      keyframe_policy_(p.kf_policy_cfg),
      imu_preint_(p.imu_cfg)
{
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
    output.stamp = stamp;

    // check our calibration is valid, if not return
    if (!calibration_.valid())
    {
        // std::cout << "[VisualInertial] Got frames but calibration is invalid!" << std::endl;
        return output;
    }

    // first frame only
    if (first_frame_)
    {
        scratch_.reserve(params_.target_features * 2, params_.target_features, gray8_left.size());
        // Upload both frames to device
        d_gray8_left_.upload(gray8_left, stream_);
        d_gray8_right_.upload(gray8_right, stream_);
        std::vector<cv::Point2f> candidates;
        std::vector<cv::Point2f> new_pts;
        candidates.reserve(static_cast<size_t>(params_.target_features) * 2);
        new_pts.reserve(params_.target_features);
        feature_detector_.detect(
            d_gray8_left_, d_mask_, candidates,
            std::max<int>(params_.target_features,
                          static_cast<int>(std::ceil(params_.target_features * params_.topup_detect_factor))));
        selectDistributedTopupPoints(
            gray8_left.size(),
            candidates,
            params_.topup_grid_scale,
            params_.target_features,
            new_pts);
        tracks_buffer_.addNewLeft(new_pts);
        first_frame_ = false;
        d_gray8_left_prev_ = d_gray8_left_.clone();
        return output;
    }

    // Upload both frames to device
    d_gray8_left_.upload(gray8_left, stream_);
    d_gray8_right_.upload(gray8_right, stream_);
    //

    tracks_buffer_.beginFrame();

    // get current tracks from buffer
    const auto &pl_prev = tracks_buffer_.pl();

    ///
    /// SECTION - Temporal tracking (FW+BW pass)
    ///
    // temporal feature tracking previous frame -> current frame
    auto &pts_fw = scratch_.pts_fw;
    auto &status_fw = scratch_.status_fw;
    tracker_temporal_.track(d_gray8_left_prev_, d_gray8_left_, pl_prev, pts_fw, status_fw);

    // temporal feature tracking current frame -> previous frame
    auto &pts_bw = scratch_.pts_bw;
    auto &status_bw = scratch_.status_bw;
    tracker_temporal_.track(d_gray8_left_, d_gray8_left_prev_, pts_fw, pts_bw, status_bw);

    // generate status vector
    // checks whether the points tracked forward and then backward are within a distance from the original points
    // generates the status vector

    uint16_t temporal_dropped = 0;
    auto &status_fb = scratch_.status_fb; // the status after applying the gate - this aligns with status_fw & status_bw
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
    auto &pts_fw_stereo = scratch_.pts_fw_stereo;
    auto &status_fw_stereo = scratch_.status_fw_stereo;
    tracker_spatial_.track(d_gray8_left_, d_gray8_right_, tracks_buffer_.pl(), pts_fw_stereo, status_fw_stereo);

    auto &pts_bw_stereo = scratch_.pts_bw_stereo;
    auto &status_bw_stereo = scratch_.status_bw_stereo;
    tracker_spatial_.track(d_gray8_right_, d_gray8_left_, pts_fw_stereo, pts_bw_stereo, status_bw_stereo);
    // std::cout << tracks_buffer_.pl().size() << " " << pts_bw_stereo.size() << std::endl;
    // gating/update db
    uint16_t stereo_dropped = 0;
    auto &keep_stereo = scratch_.keep_stereo;
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
    auto &object_pts = scratch_.object_pts;
    auto &image_pts = scratch_.image_pts;
    auto &buf_idx = scratch_.buf_idx; // map: eligible index -> buffer index (needed for B-style gating)

    tracks_buffer_.gatherPnP(object_pts, image_pts, &buf_idx);

    if (object_pts.size() < 6)
        std::cout << "[PNP] not enough points!" << std::endl;

    bool have_pose_update = false;
    size_t pnp_tracks_for_policy = object_pts.size();
    const Eigen::Isometry3d pose_prev = vo_pose_abs_;
    Eigen::Isometry3d T_Ck_Cref = Eigen::Isometry3d::Identity();
    cv::Mat rvec_guess;
    cv::Mat tvec_guess;
    const bool have_anchor_guess = have_ref_kf_;

    if (have_anchor_guess)
    {
        const Eigen::Isometry3d T_Cprev_Cref = pose_prev.inverse() * ref_kf_pose_abs_;
        isometryToRvecTvec(T_Cprev_Cref, rvec_guess, tvec_guess);
    }

    if (object_pts.size() >= 6)
    {
        // -----------------------------
        // 4) solvePnPRansac (AP3P) + refineLM on inliers
        // -----------------------------
        cv::Mat rvec, tvec;
        auto &inliers = scratch_.inliers;

        const int iterationsCount = params_.pnp_iterations_count;
        const float reprojErrorPx = params_.pnp_reproj_error_px;
        const double confidence = params_.pnp_confidence;

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
        {
            std::cout << "[PNP] Error!" << std::endl;
        }
        else
        {
            // Refine using LM on inliers.
            auto &obj_in = scratch_.obj_in;
            auto &img_in = scratch_.img_in;
            obj_in.clear();
            img_in.clear();
            obj_in.reserve(inliers.size());
            img_in.reserve(inliers.size());

            for (int j : inliers)
            {
                obj_in.push_back(object_pts[(size_t)j]);
                img_in.push_back(image_pts[(size_t)j]);
            }

            cv::Mat rvec_iter = have_anchor_guess ? rvec_guess.clone() : rvec.clone();
            cv::Mat tvec_iter = have_anchor_guess ? tvec_guess.clone() : tvec.clone();

            bool iter_ok = cv::solvePnP(
                obj_in, img_in, K_left, cv::Mat(),
                rvec_iter, tvec_iter,
                true,
                cv::SOLVEPNP_ITERATIVE);

            if (!iter_ok && have_anchor_guess)
            {
                rvec_iter = rvec.clone();
                tvec_iter = tvec.clone();
                iter_ok = cv::solvePnP(
                    obj_in, img_in, K_left, cv::Mat(),
                    rvec_iter, tvec_iter,
                    true,
                    cv::SOLVEPNP_ITERATIVE);
            }

            if (iter_ok)
            {
                rvec = rvec_iter;
                tvec = tvec_iter;
            }

            cv::TermCriteria crit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                  params_.pnp_refine_max_iters,
                                  params_.pnp_refine_eps);
            cv::solvePnPRefineLM(obj_in, img_in, K_left, cv::Mat(), rvec, tvec, crit);

            // -----------------------------
            // 5) Gate DB based on PnP inliers (B-style: drop eligible outliers only)
            // -----------------------------
            tracks_buffer_.gateByPnPInliers(buf_idx, inliers);
            pnp_tracks_for_policy = inliers.size();

            // -----------------------------
            // 6) Convert (rvec,tvec) -> Eigen::Isometry3d T_Ck_Cref
            // -----------------------------
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

            T_Ck_Cref.linear() = R;
            T_Ck_Cref.translation() = t;
            have_pose_update = true;

            if (have_ref_kf_)
                vo_pose_abs_ = ref_kf_pose_abs_ * T_Ck_Cref.inverse();
        }
    }
    output.vo_pose_rel = pose_prev.inverse() * vo_pose_abs_;

    // add the tracks up to this point to the output, these are the tracks that helped steer the current update
    // these tracks we born at least on the last frame, and passed our temporal + stereo gates, and the pnp gate
    output.tracks = tracks_buffer_.pl();
    //
    // SECTION - KEYFRAME EVALUATION AND CREATION
    //
    // evaluate keyfrmae generation
    // if generate keyframe -> create a keyframe and put in return struct

    // 1) Ask the policy if we should create a keyframe
    KeyframePolicy::Input kf_in;
    kf_in.t_s = stamp;
    kf_in.T_WC = vo_pose_abs_;
    kf_in.pose_valid = have_pose_update;

    kf_in.track_ids = tracks_buffer_.ids().data();
    kf_in.num_tracks = tracks_buffer_.ids().size();
    kf_in.num_pnp_tracks = pnp_tracks_for_policy;

    KeyframePolicy::Decision dec = keyframe_policy_.evaluate(kf_in);

    // 2) If yes: build the keyframe event (1–10 Hz, copying vectors is fine)
    if (dec.make_keyframe)
    {
        const size_t N = tracks_buffer_.size();
        const auto &pl = tracks_buffer_.pl();
        const auto &pr = tracks_buffer_.pr();
        const auto &has_r = tracks_buffer_.hasRight();
        auto &X_curr = scratch_.X_curr;
        auto &valid_X = scratch_.valid_X;
        X_curr.resize(N);
        valid_X.resize(N);

        const float fx = calibration_.left.fx();
        const float fy = calibration_.left.fy();
        const float cx = calibration_.left.cx();
        const float cy = calibration_.left.cy();
        const float baseline = calibration_.baseline;

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
                fx, fy, cx, cy, baseline,
                X);

            valid_X[i] = ok ? uint8_t{1} : uint8_t{0};
            X_curr[i] = ok ? X : cv::Point3f(0.f, 0.f, 0.f);
        }

        // Refresh the anchor landmarks only when a new keyframe is accepted.
        tracks_buffer_.setPrev3DAll(X_curr, &valid_X);
        ref_kf_pose_abs_ = vo_pose_abs_;
        have_ref_kf_ = true;

        KeyframeEvent ev;
        ev.kf_id = next_kf_id_++;
        ev.t_start = timestamp_last_kf_;
        ev.t_end = stamp;

        const Eigen::Isometry3d T_BC = params_.T_BC; ///* your calibrated Body<-CamOptical (rot+trans) */;
        const Eigen::Isometry3d T_CB = T_BC.inverse();
        const Eigen::Isometry3d T_WB_kf = T_BC * vo_pose_abs_ * T_CB;

        ev.T_WC = T_WB_kf; // <-- STORE World<-Body in the KF (reuse field for now)
        if (have_last_kf_body_pose_)
        {
            ev.has_vo_between = true;
            ev.T_Bkm1_Bk = last_kf_body_pose_.inverse() * T_WB_kf;
        }
        else
        {
            ev.has_vo_between = false;
            ev.T_Bkm1_Bk = Eigen::Isometry3d::Identity();
        }
        // v.T_WC = vo_pose_abs_;

        // Tracks in deterministic order (same order across ids/pl/pr)
        ev.ids = tracks_buffer_.ids();
        ev.pl = tracks_buffer_.pl();
        ev.pr = tracks_buffer_.pr();
        ev.has_r = tracks_buffer_.hasRight(); // or whatever you name it

        // 4) Inform policy that the keyframe was accepted/created
        //    (policy stores last-kf pose/time + last-kf ids for overlap logic)
        keyframe_policy_.onKeyframeCreated(ev.kf_id, ev.t_end, vo_pose_abs_, ev.ids);
        output.kf_trigger = true;
        last_kf_body_pose_ = T_WB_kf;
        have_last_kf_body_pose_ = true;

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

        // instead of building IMU here:
        submitPendingKeyframe_(std::move(ev));

        timestamp_last_kf_ = stamp; // update last stamp
        prev_kf_id_ = ev.kf_id;
    }

    ///
    /// SECTION - TOPUP
    /// Add new tracks after KF evaluation so fresh points do not hide the need for a new anchor keyframe.
    const size_t usable_track_count = countSetFlags(tracks_buffer_.hasRight());
    const int usable_deficit =
        static_cast<int>(params_.target_features) - static_cast<int>(usable_track_count);
    const int total_capacity =
        static_cast<int>(params_.max_total_tracks) - static_cast<int>(tracks_buffer_.size());

    if (usable_deficit > 0 && total_capacity > 0)
    {
        buildExclusionMaskDownsampledCPU(
            d_gray8_left_.size(),
            tracks_buffer_.pl(),
            params_.mask_scale,
            mask_small_cpu_,
            mask_full_cpu_);

        d_mask_.upload(mask_full_cpu_, stream_);

        const int refill_goal = std::max(
            usable_deficit,
            static_cast<int>(std::ceil(usable_deficit * params_.topup_burst_factor)));
        const int topup_limit = std::min(refill_goal, total_capacity);
        const int detect_limit = std::min(
            total_capacity,
            std::max(
                topup_limit,
                static_cast<int>(std::ceil(topup_limit * params_.topup_detect_factor))));

        auto &candidate_pts = scratch_.candidate_pts;
        candidate_pts.clear();
        candidate_pts.reserve(detect_limit);

        auto &new_pts = scratch_.new_pts;
        new_pts.clear();
        new_pts.reserve(topup_limit);

        feature_detector_.detect(d_gray8_left_, d_mask_, candidate_pts, detect_limit);
        selectDistributedTopupPoints(
            d_gray8_left_.size(),
            candidate_pts,
            params_.topup_grid_scale,
            static_cast<size_t>(topup_limit),
            new_pts);

        if (!new_pts.empty())
            tracks_buffer_.addNewLeft(new_pts);
    }

    // debug

    if (dec.make_keyframe)
    {
        std::cout << keyframeDecisionDebugLine(dec) << "\n";
    }

    // SECTION - FILL OUTPUT POSE ABS

    const Eigen::Isometry3d T_BC = params_.T_BC; ///* your calibrated Body<-CamOptical (rot+trans) */;
    const Eigen::Isometry3d T_CB = T_BC.inverse();

    output.vo_pose_abs = T_BC * vo_pose_abs_ * T_CB;

    ///
    /// SECTION - UPDATE BUFFERS
    ///

    d_gray8_left_prev_.create(d_gray8_left_.size(), d_gray8_left_.type());
    d_gray8_left_.copyTo(d_gray8_left_prev_, stream_);

    scratch_.clearPerFrame();

    return output;
}

void VisualInertial::submitPendingKeyframe_(KeyframeEvent &&ev)
{
    std::lock_guard<std::mutex> lk(kf_mtx_);
    pending_kfs_.push_back(std::move(ev));
    while (pending_kfs_.size() > params_.kf_pending_queue_max)
        pending_kfs_.pop_front();
}

bool VisualInertial::tryPopFinalizedKeyframe(KeyframeEvent &out)
{
    std::lock_guard<std::mutex> lk(kf_mtx_);
    if (ready_kfs_.empty())
        return false;
    out = std::move(ready_kfs_.front());
    ready_kfs_.pop_front();
    return true;
}

bool VisualInertial::hasImuCoverageForFinalize_(double t1) const
{
    // Margin-based coverage check (less brittle than exact comparisons).
    // Adjust names if your ImuPreintegrator API differs.
    return imu_preint_.size() >= 2 &&
           imu_preint_.newestTime() >= (t1 - params_.imu_coverage_margin_s);
}

bool VisualInertial::tryFinalizeOne()
{
    KeyframeEvent ev;

    // Peek the next pending KF (do NOT pop yet)
    {
        std::lock_guard<std::mutex> lk(kf_mtx_);
        if (pending_kfs_.empty())
            return false;
        ev = pending_kfs_.front(); // copy is fine at 1–10 Hz
    }

    // 1) First finalized KF: emit without IMU (bootstrap)
    if (!have_last_finalized_)
    {
        ev.has_imu = false;
        ev.pim_bytes.clear();

        std::lock_guard<std::mutex> lk(kf_mtx_);
        if (!pending_kfs_.empty() && pending_kfs_.front().kf_id == ev.kf_id)
            pending_kfs_.pop_front();
        ready_kfs_.push_back(std::move(ev));
        while (ready_kfs_.size() > params_.kf_ready_queue_max)
            ready_kfs_.pop_front();

        have_last_finalized_ = true;
        last_finalized_kf_id_ = ready_kfs_.back().kf_id;
        last_finalized_t_end_ = ready_kfs_.back().t_end;

        std::cout << "[KFfinal] first KF kf_id=" << last_finalized_kf_id_
                  << " (no IMU)\n";
        return true;
    }

    // 2) Chain sanity
    const uint64_t prev_id = last_finalized_kf_id_;
    const double t0 = ev.t_start;
    const double t1 = ev.t_end;

    const bool ids_ok = (ev.kf_id == prev_id + 1);
    const bool times_ok = (std::abs(t0 - last_finalized_t_end_) < 1e-3) && (t1 > t0);

    if (!ids_ok || !times_ok)
    {
        // Emit without IMU and reset anchor
        ev.has_imu = false;
        ev.pim_bytes.clear();

        std::lock_guard<std::mutex> lk(kf_mtx_);
        if (!pending_kfs_.empty() && pending_kfs_.front().kf_id == ev.kf_id)
            pending_kfs_.pop_front();
        ready_kfs_.push_back(std::move(ev));
        while (ready_kfs_.size() > params_.kf_ready_queue_max)
            ready_kfs_.pop_front();

        last_finalized_kf_id_ = ready_kfs_.back().kf_id;
        last_finalized_t_end_ = ready_kfs_.back().t_end;

        std::cout << std::fixed << std::setprecision(6)
                  << "[KFfinal] chain break: kf_id=" << last_finalized_kf_id_
                  << " ids_ok=" << (ids_ok ? 1 : 0)
                  << " times_ok=" << (times_ok ? 1 : 0)
                  << " t0=" << t0
                  << " last_t_end=" << last_finalized_t_end_
                  << " t1=" << t1 << "\n";
        return true;
    }

    // 3) If not enough IMU yet, keep pending and try later
    // (Use your existing coverage function if you have it; this margin-based check is what you were using.)
    const bool have_cov = hasImuCoverageForFinalize_(t1);
    if (!have_cov)
        return false;

    // 4) Attempt to build packet. If it fails due to timing, DO NOT drop KF; retry later.
    auto pkt_opt = imu_preint_.buildAndConsume(prev_id, t0, ev.kf_id, t1);

    if (!pkt_opt || !pkt_opt->valid)
    {
        // If we *thought* we had coverage but build still failed, it's almost always boundary timing.
        // Treat as "not ready yet" and retry on the next worker wakeup.
        // (If you want a hard timeout, you can add it later.)
        std::cout << std::fixed << std::setprecision(6)
                  << "[KFfinal] pkt not ready, will retry: kf_id=" << ev.kf_id
                  << " t0=" << t0 << " t1=" << t1
                  << " imu_new=" << imu_preint_.newestTime()
                  << " imu_old=" << imu_preint_.oldestTime()
                  << " imu_sz=" << imu_preint_.size()
                  << "\n";
        return false;
    }

    // 5) Success: now pop pending and publish finalized
    ev.has_imu = true;
    ev.pim_bytes = std::move(pkt_opt->bytes);

    {
        std::lock_guard<std::mutex> lk(kf_mtx_);
        if (!pending_kfs_.empty() && pending_kfs_.front().kf_id == ev.kf_id)
            pending_kfs_.pop_front();

        ready_kfs_.push_back(std::move(ev));
        while (ready_kfs_.size() > params_.kf_ready_queue_max)
            ready_kfs_.pop_front();

        last_finalized_kf_id_ = ready_kfs_.back().kf_id;
        last_finalized_t_end_ = ready_kfs_.back().t_end;
    }

    std::cout << "[KFfinal] finalized kf_id=" << last_finalized_kf_id_
              << " imu_bytes=" << ready_kfs_.back().pim_bytes.size() << "\n";

    return true;
}
