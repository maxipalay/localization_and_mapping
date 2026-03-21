#pragma once

#include <cstdlib>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <utility>

namespace online_mapping_logger
{

template <typename TValue>
class StampBuffer
{
public:
  void add(int64_t stamp_ns, TValue value, int64_t buffer_duration_ns)
  {
    buffer_[stamp_ns] = std::move(value);
    prune(stamp_ns - buffer_duration_ns);
  }

  std::optional<std::pair<int64_t, TValue>> findNearest(
    int64_t target_stamp_ns,
    int64_t tolerance_ns) const
  {
    if (buffer_.empty()) {
      return std::nullopt;
    }

    auto best_it = buffer_.end();
    int64_t best_abs_dt = std::numeric_limits<int64_t>::max();

    auto consider = [&](typename std::map<int64_t, TValue>::const_iterator it) {
      if (it == buffer_.end()) {
        return;
      }
      const int64_t dt = std::llabs(it->first - target_stamp_ns);
      if (dt < best_abs_dt) {
        best_abs_dt = dt;
        best_it = it;
      }
    };

    const auto lb = buffer_.lower_bound(target_stamp_ns);
    consider(lb);
    if (lb != buffer_.begin()) {
      consider(std::prev(lb));
    }

    if (best_it == buffer_.end() || best_abs_dt > tolerance_ns) {
      return std::nullopt;
    }

    return std::make_optional(std::make_pair(best_it->first, best_it->second));
  }

private:
  void prune(int64_t min_stamp_ns)
  {
    while (!buffer_.empty() && buffer_.begin()->first < min_stamp_ns) {
      buffer_.erase(buffer_.begin());
    }
  }

  std::map<int64_t, TValue> buffer_;
};

}  // namespace online_mapping_logger
