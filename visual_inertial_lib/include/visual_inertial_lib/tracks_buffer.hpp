#pragma once
#include <opencv2/core.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <mutex>
#include <vector>

class TracksBuffer {
public:
  using TrackId = uint32_t;

  explicit TracksBuffer(size_t reserve_n = 0) { reserve(reserve_n); }

  void reserve(size_t n);

  // Called once per frame
  void beginFrame();          // clears right-valid flags (and optionally depth-valid)
  void clear();               // drop everything (rare)

  size_t size() const { return ids_.size(); }
  bool empty() const { return ids_.empty(); }

  // Deterministic order access (no copies)
  const std::vector<TrackId>& ids()   const { return ids_; }
  const std::vector<cv::Point2f>& pl() const { return pl_; }
  const std::vector<cv::Point2f>& pr() const { return pr_; }
  const std::vector<uint8_t>& hasRight() const { return has_r_; }

  // refresh tracked left points (same ordering)
  void setLeftInPlace(const std::vector<cv::Point2f>& pl_new);

  // apply gating mask in same order as pl()
  // keep[i] != 0 -> keep. Stable compaction.
  void applyKeepMask(const std::vector<uint8_t>& keep);

  // right points come later: fill partial or full
  void setRightAll(const std::vector<cv::Point2f>& pr_new,
                   const std::vector<uint8_t>* valid = nullptr);

  void setRightForIndices(const std::vector<int>& indices,
                          const std::vector<cv::Point2f>& pr_values);

  // top up with new tracks (auto IDs)
  // appends in order of pts in pl_new
  void addNewLeft(const std::vector<cv::Point2f>& pl_new,
                  std::vector<TrackId>* out_ids = nullptr);


private:
  TrackId next_id_ = 1;

  std::vector<TrackId> ids_;
  std::vector<cv::Point2f> pl_;
  std::vector<cv::Point2f> pr_;
  std::vector<uint8_t> has_r_;

};
