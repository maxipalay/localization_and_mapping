// TracksBuffer.cpp
#include "visual_inertial_lib/tracks_buffer.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>

TracksBuffer::TracksBuffer(size_t reserve_n)
{
    reserve(reserve_n);
}

void TracksBuffer::reserve(size_t n)
{
    ids_.reserve(n);
    pl_.reserve(n);
    pr_.reserve(n);
    has_r_.reserve(n);
    X_prev_.reserve(n);
    has_X_prev_.reserve(n);
}

void TracksBuffer::beginFrame()
{
    std::fill(has_r_.begin(), has_r_.end(), uint8_t{0});
}

void TracksBuffer::clear()
{
    ids_.clear();
    pl_.clear();
    pr_.clear();
    has_r_.clear();
    X_prev_.clear();
    has_X_prev_.clear();
    // next_id_ intentionally not reset (monotonic IDs are usually helpful).
}

size_t TracksBuffer::size() const { return ids_.size(); }
bool TracksBuffer::empty() const { return ids_.empty(); }

const std::vector<TracksBuffer::TrackId> &TracksBuffer::ids() const { return ids_; }
const std::vector<cv::Point2f> &TracksBuffer::pl() const { return pl_; }
const std::vector<cv::Point2f> &TracksBuffer::pr() const { return pr_; }
const std::vector<uint8_t> &TracksBuffer::hasRight() const { return has_r_; }

const std::vector<cv::Point3f> &TracksBuffer::Xprev() const { return X_prev_; }
const std::vector<uint8_t> &TracksBuffer::hasXprev() const { return has_X_prev_; }

void TracksBuffer::setLeftInPlace(const std::vector<cv::Point2f> &pl_new)
{
    assert(pl_new.size() == pl_.size());
    if (!pl_new.empty())
    {
        std::copy(pl_new.begin(), pl_new.end(), pl_.begin());
    }
}

void TracksBuffer::applyKeepMask(const std::vector<uint8_t> &keep)
{
    const size_t n = ids_.size();
    assert(keep.size() == n);
    assert(pl_.size() == n && pr_.size() == n && has_r_.size() == n);
    assert(X_prev_.size() == n && has_X_prev_.size() == n);

    size_t w = 0;
    for (size_t i = 0; i < n; ++i)
    {
        if (!keep[i])
            continue;

        if (w != i)
        {
            ids_[w] = ids_[i];
            pl_[w] = pl_[i];
            pr_[w] = pr_[i];
            has_r_[w] = has_r_[i];
            X_prev_[w] = X_prev_[i];
            has_X_prev_[w] = has_X_prev_[i];
        }
        ++w;
    }

    ids_.resize(w);
    pl_.resize(w);
    pr_.resize(w);
    has_r_.resize(w);
    X_prev_.resize(w);
    has_X_prev_.resize(w);
}

void TracksBuffer::setRightInPlace(const std::vector<cv::Point2f> &pr_new)
{
    const size_t n = ids_.size();
    assert(pr_new.size() == n);
    assert(pr_.size() == n);
    assert(has_r_.size() == n);

    if (n)
    {
        std::copy(pr_new.begin(), pr_new.end(), pr_.begin());
    }

    // If you’re using right as “per-frame refreshed”, mark as valid by default here.
    // If you prefer to control validity externally too, delete this fill and just
    // manage has_r_ via a separate setter.
    std::fill(has_r_.begin(), has_r_.end(), uint8_t{1});
}

void TracksBuffer::setPrev3DAll(const std::vector<cv::Point3f> &X_prev_new,
                                const std::vector<uint8_t> *valid)
{
    const size_t n = ids_.size();
    assert(X_prev_new.size() == n);
    assert(X_prev_.size() == n && has_X_prev_.size() == n);

    if (n)
    {
        std::copy(X_prev_new.begin(), X_prev_new.end(), X_prev_.begin());
    }

    if (valid)
    {
        assert(valid->size() == n);
        for (size_t i = 0; i < n; ++i)
        {
            has_X_prev_[i] = ((*valid)[i] != 0) ? uint8_t{1} : uint8_t{0};
        }
    }
    else
    {
        std::fill(has_X_prev_.begin(), has_X_prev_.end(), uint8_t{1});
    }
}

void TracksBuffer::setPrev3DForIndices(const std::vector<int> &indices,
                                       const std::vector<cv::Point3f> &X_values)
{
    assert(indices.size() == X_values.size());
    const int n = static_cast<int>(ids_.size());

    for (size_t k = 0; k < indices.size(); ++k)
    {
        const int idx = indices[k];
        assert(idx >= 0 && idx < n);
        X_prev_[static_cast<size_t>(idx)] = X_values[k];
        has_X_prev_[static_cast<size_t>(idx)] = uint8_t{1};
    }
}

void TracksBuffer::invalidatePrev3DAll()
{
    std::fill(has_X_prev_.begin(), has_X_prev_.end(), uint8_t{0});
}

void TracksBuffer::addNewLeft(const std::vector<cv::Point2f> &pl_new,
                              std::vector<TrackId> *out_ids)
{
    if (pl_new.empty())
        return;

    const size_t add_n = pl_new.size();
    const size_t new_size = ids_.size() + add_n;

    // Grow capacity aggressively to avoid reallocations in hot path.
    if (ids_.capacity() < new_size)
    {
        const size_t cap = std::max(new_size, std::max<size_t>(64, ids_.capacity() * 2));
        reserve(cap);
    }

    if (out_ids)
    {
        out_ids->clear();
        out_ids->reserve(add_n);
    }

    for (size_t i = 0; i < add_n; ++i)
    {
        const TrackId id = next_id_++;

        ids_.push_back(id);
        pl_.push_back(pl_new[i]);

        // Right unknown until setRight*
        pr_.push_back(cv::Point2f(0.f, 0.f));
        has_r_.push_back(uint8_t{0});

        // Prev-3D unknown until setPrev3D*
        X_prev_.push_back(cv::Point3f(0.f, 0.f, 0.f));
        has_X_prev_.push_back(uint8_t{0});

        if (out_ids)
            out_ids->push_back(id);
    }
}

void TracksBuffer::gatherPnP(std::vector<cv::Point3f> &object_pts,
                             std::vector<cv::Point2f> &image_pts,
                             std::vector<int> *out_indices) const
{
    object_pts.clear();
    image_pts.clear();
    if (out_indices)
        out_indices->clear();

    const size_t n = ids_.size();
    object_pts.reserve(n);
    image_pts.reserve(n);
    if (out_indices)
        out_indices->reserve(n);

    for (size_t i = 0; i < n; ++i)
    {
        if (!has_X_prev_[i])
            continue;
        object_pts.push_back(X_prev_[i]);
        image_pts.push_back(pl_[i]);
        if (out_indices)
            out_indices->push_back(static_cast<int>(i));
    }
}

void TracksBuffer::gateByPnPInliers(const std::vector<int>& candidate_buf_idx,
                                                const std::vector<int>& inliers) {
  // candidate_buf_idx.size() == number of correspondences used in PnP (eligible set)
  // inliers are indices into [0, candidate_buf_idx.size()) returned by solvePnPRansac

  const size_t N = ids_.size();

  // Keep everything by default (pending tracks survive)
  std::vector<uint8_t> keep(N, uint8_t{1});

  // Mark candidate inliers in candidate-index space
  std::vector<uint8_t> cand_is_inlier(candidate_buf_idx.size(), uint8_t{0});
  for (int j : inliers) {
    if (j >= 0 && static_cast<size_t>(j) < cand_is_inlier.size()) {
      cand_is_inlier[static_cast<size_t>(j)] = uint8_t{1};
    }
  }

  // Drop candidate outliers (map candidate -> buffer index)
  for (size_t j = 0; j < candidate_buf_idx.size(); ++j) {
    if (cand_is_inlier[j]) continue;
    const int bi = candidate_buf_idx[j];
    if (bi >= 0 && static_cast<size_t>(bi) < N) {
      keep[static_cast<size_t>(bi)] = uint8_t{0};
    }
  }

  applyKeepMask(keep);
}
