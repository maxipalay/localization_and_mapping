// TracksBuffer.hpp
#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <vector>

class TracksBuffer
{
public:
    using TrackId = uint32_t;

    explicit TracksBuffer(size_t reserve_n = 0);

    void reserve(size_t n);

    // called once per frame, right observations are per-frame and get refreshed; prev-3D stays.
    void beginFrame();

    // drop everything
    void clear();

    size_t size() const;
    bool empty() const;

    // deterministic order access - zero copy
    const std::vector<TrackId> &ids() const;
    const std::vector<cv::Point2f> &pl() const;
    const std::vector<cv::Point2f> &pr() const;
    const std::vector<uint8_t> &hasRight() const;

    // prev-frame 3D cache aligned to tracks used for PnP with current pl_
    const std::vector<cv::Point3f> &Xprev() const;
    const std::vector<uint8_t> &hasXprev() const;

    // refresh tracked left points (same ordering)
    void setLeftInPlace(const std::vector<cv::Point2f> &pl_new);

    // apply gating mask in same order as pl(); stable compaction
    void applyKeepMask(const std::vector<uint8_t> &keep);

    // right points come later: fill partial or full
    // Refresh tracked right points (same ordering as tracks.pl()).
    // Does NOT gate/drop anything
    void setRightInPlace(const std::vector<cv::Point2f> &pr_new);

    // Prev-frame 3D comes from "previous frame" triangulation
    // computed elsewhere. This buffer just stores it aligned to tracks
    void setPrev3DAll(const std::vector<cv::Point3f> &X_prev_new,
                      const std::vector<uint8_t> *valid = nullptr);

    void setPrev3DForIndices(const std::vector<int> &indices,
                             const std::vector<cv::Point3f> &X_values);

    void invalidatePrev3DAll();

    // top up with new tracks (auto IDs). Appends in order of pts in pl_new.
    void addNewLeft(const std::vector<cv::Point2f> &pl_new,
                    std::vector<TrackId> *out_ids = nullptr);

    // helper: gather only valid PnP correspondences
    void gatherPnP(std::vector<cv::Point3f> &object_pts,
                   std::vector<cv::Point2f> &image_pts,
                   std::vector<int> *out_indices = nullptr) const;

private:
    TrackId next_id_ = 1;

    // SoA storage
    std::vector<TrackId> ids_;
    std::vector<cv::Point2f> pl_;
    std::vector<cv::Point2f> pr_;
    std::vector<uint8_t> has_r_;

    // prev-frame 3D cache aligned with tracks for PnP this frame
    std::vector<cv::Point3f> X_prev_;
    std::vector<uint8_t> has_X_prev_;
};
