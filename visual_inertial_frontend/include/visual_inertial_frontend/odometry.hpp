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
};
