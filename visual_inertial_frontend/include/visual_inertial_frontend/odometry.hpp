#pragma once

#include "visual_inertial_frontend/tracks_buffer.hpp"
#include "visual_inertial_frontend/feature_tracker.hpp"
#include "visual_inertial_frontend/feature_detector.hpp"
#include "visual_inertial_frontend/types.hpp"
#include "visual_inertial_frontend/keyframe_policy.hpp"
#include "visual_inertial_frontend/preintegrator.hpp"
#include "visual_inertial_common/types.hpp"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

#include <opencv2/core/cuda.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <optional>

class VisualInertial
{
public:
    struct Params
    {
        uint16_t target_features = 500;
        float stereo_epi_eps_y = 1.5;
        float stereo_disp_min = 3.0;
        float stereo_disp_max = 255.0;
        float fb_thr2 = 1.5 * 1.5;

        static Eigen::Isometry3d default_T_BC()
        {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.linear() << 0.0, 0.0, 1.0,
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0;
            T.translation() = Eigen::Vector3d(0.0, 0.05, 0.0);
            return T;
        }

        Eigen::Isometry3d T_BC = default_T_BC();

        double imu_coverage_margin_s = 0.05;
        size_t kf_ready_queue_max = 30;
        size_t kf_pending_queue_max = 30;

        // Mask generation for top-up; higher = coarser blocks
        int mask_scale = 20;
        int topup_grid_scale = 20;
        uint16_t max_total_tracks = 1000;
        double topup_burst_factor = 2.0;
        double topup_detect_factor = 3.0;

        // PnP tuning
        int pnp_iterations_count = 100;
        float pnp_reproj_error_px = 2.0f;
        double pnp_confidence = 0.999;
        int pnp_refine_max_iters = 30;
        double pnp_refine_eps = 1e-6;

        // Keyframe policy configuration
        KeyframePolicy::Config kf_policy_cfg{};

        // IMU preintegration configuration
        ImuPreintegratorConfig imu_cfg{};
    };

    VisualInertial()
        : VisualInertial(Params{})
    {
    }

    VisualInertial(const Params &p);

    FrameResult processStereo(const cv::Mat &gray8_left,
                              const cv::Mat &gray8_right, double stamp);

    void processImu(const ImuSample &sample);

    void setCalibration(const CameraRig &calibration)
    {
        calibration_ = calibration;
    }

    // Thread-safe IMU helpers for the node's KF-finalizer worker thread.
    // These MUST NOT touch any VO state; they only forward to imu_preint_ (which has its own mutex).
    bool hasImuCoverage(double t_s) const
    {
        return imu_preint_.hasCoverage(t_s);
    }

    std::optional<PreintegratedImuPacket> buildImuPacket(uint64_t prev_kf_id,
                                                         double t0_s,
                                                         uint64_t kf_id,
                                                         double t1_s)
    {
        return imu_preint_.buildAndConsume(prev_kf_id, t0_s, kf_id, t1_s);
    }

    // Optional for later: backend bias feedback
    void setImuBias(const ImuBias &b)
    {
        imu_preint_.setBias(b);
    }

    // Called by the node-owned worker thread.
    // Returns true if it finalized ONE pending keyframe and moved it into the ready queue.
    // Returns false if nothing pending OR IMU coverage isn't ready yet.
    bool tryFinalizeOne();

    // Called by the node worker thread (or any thread): pop a finalized keyframe to publish.
    bool tryPopFinalizedKeyframe(KeyframeEvent &out);

private:
    Params params_;
    bool first_frame_ = true;

    CameraRig calibration_;
    TracksBuffer tracks_buffer_;
    FeatureDetector feature_detector_;
    FeatureTracker tracker_temporal_;
    FeatureTracker tracker_spatial_;
    KeyframePolicy keyframe_policy_;
    ImuPreintegrator imu_preint_;

    cv::cuda::Stream stream_{};
    cv::cuda::GpuMat d_mask_{}; // mask on device for feature top-up
    cv::cuda::GpuMat d_gray8_left_{};
    cv::cuda::GpuMat d_gray8_right_{};
    cv::cuda::GpuMat d_gray8_left_prev_{};

    Eigen::Isometry3d vo_pose_abs_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d ref_kf_pose_abs_ = Eigen::Isometry3d::Identity();
    bool have_ref_kf_ = false;

    uint64_t next_kf_id_ = 0;
    uint64_t prev_kf_id_ = 0;

    double timestamp_last_kf_ = 0.0;

    // --- keyframe finalization queues (library-owned state, node-owned thread) ---
    mutable std::mutex kf_mtx_;
    std::deque<KeyframeEvent> pending_kfs_;
    std::deque<KeyframeEvent> ready_kfs_;

    bool have_last_finalized_{false};
    uint64_t last_finalized_kf_id_{0};
    double last_finalized_t_end_{0.0};

    // Internal helpers
    void submitPendingKeyframe_(KeyframeEvent &&ev);
    bool hasImuCoverageForFinalize_(double t1) const;

    struct Scratch
    {
        // Temporal LK
        std::vector<cv::Point2f> pts_fw, pts_bw;
        std::vector<uint8_t> status_fw, status_bw, status_fb;

        // Stereo LK
        std::vector<cv::Point2f> pts_fw_stereo, pts_bw_stereo;
        std::vector<uint8_t> status_fw_stereo, status_bw_stereo, keep_stereo;

        // PnP
        std::vector<cv::Point3f> object_pts;
        std::vector<cv::Point2f> image_pts;
        std::vector<int> buf_idx;
        std::vector<int> inliers;
        std::vector<cv::Point3f> obj_in;
        std::vector<cv::Point2f> img_in;

        // Triangulation / 3D buffer
        std::vector<cv::Point3f> X_curr;
        std::vector<uint8_t> valid_X;

        // Top-up
        std::vector<cv::Point2f> new_pts;
        std::vector<cv::Point2f> candidate_pts;

        // CPU mask reuse
        cv::Mat cpu_mask;

        void reserve(size_t n_tracks, size_t n_new, cv::Size img_sz)
        {
            pts_fw.reserve(n_tracks);
            pts_bw.reserve(n_tracks);
            status_fw.reserve(n_tracks);
            status_bw.reserve(n_tracks);
            status_fb.reserve(n_tracks);

            pts_fw_stereo.reserve(n_tracks);
            pts_bw_stereo.reserve(n_tracks);
            status_fw_stereo.reserve(n_tracks);
            status_bw_stereo.reserve(n_tracks);
            keep_stereo.reserve(n_tracks);

            object_pts.reserve(n_tracks);
            image_pts.reserve(n_tracks);
            buf_idx.reserve(n_tracks);
            inliers.reserve(n_tracks);
            obj_in.reserve(n_tracks);
            img_in.reserve(n_tracks);

            X_curr.reserve(n_tracks);
            valid_X.reserve(n_tracks);

            new_pts.reserve(n_new);
            candidate_pts.reserve(n_new * 2);

            if (cpu_mask.empty() || cpu_mask.size() != img_sz)
            {
                cpu_mask.create(img_sz, CV_8U);
            }
        }

        void clearPerFrame()
        {
            // Don’t shrink. Just clear sizes.
            pts_fw.clear();
            pts_bw.clear();
            status_fw.clear();
            status_bw.clear();
            status_fb.clear();

            pts_fw_stereo.clear();
            pts_bw_stereo.clear();
            status_fw_stereo.clear();
            status_bw_stereo.clear();
            keep_stereo.clear();

            object_pts.clear();
            image_pts.clear();
            buf_idx.clear();
            inliers.clear();
            obj_in.clear();
            img_in.clear();

            X_curr.clear();
            valid_X.clear();

            new_pts.clear();
            candidate_pts.clear();
        }
    };
    Scratch scratch_;

    // Mask buffers
    cv::Mat mask_small_cpu_;
    cv::Mat mask_full_cpu_; // full-res CPU mask
};
