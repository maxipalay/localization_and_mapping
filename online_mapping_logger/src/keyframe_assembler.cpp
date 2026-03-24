#include "online_mapping_logger/keyframe_assembler.hpp"

namespace online_mapping_logger
{

KeyframeAssembler::KeyframeAssembler(LoggerConfig config)
: config_(std::move(config))
{
}

AssemblerUpdate KeyframeAssembler::addRgb(ImageMsg::ConstSharedPtr msg)
{
  rgb_buffer_.add(stampToNs(msg->header.stamp), msg, config_.buffer_duration_ns);
  return collectUpdate_();
}

AssemblerUpdate KeyframeAssembler::addDepth(ImageMsg::ConstSharedPtr msg)
{
  depth_buffer_.add(stampToNs(msg->header.stamp), msg, config_.buffer_duration_ns);
  return collectUpdate_();
}

AssemblerUpdate KeyframeAssembler::addKeyframe(KeyframeMsg::ConstSharedPtr msg, int64_t now_ns)
{
  PendingKeyframe pending;
  pending.keyframe = *msg;
  pending.keyframe_stamp_ns = stampToNs(msg->header.stamp);
  pending.created_at_ns = now_ns;

  const auto opt_it = optimization_buffer_.find(msg->kf_id);
  if (opt_it != optimization_buffer_.end()) {
    pending.have_opt_result = true;
    pending.opt_result = opt_it->second;
    optimization_buffer_.erase(opt_it);
  }

  pending_[msg->kf_id] = std::move(pending);
  return collectUpdate_();
}

AssemblerUpdate KeyframeAssembler::addOptimizationResult(OptimizationResultMsg::ConstSharedPtr msg)
{
  const auto pending_it = pending_.find(msg->kf_id);
  if (pending_it != pending_.end()) {
    pending_it->second.have_opt_result = true;
    pending_it->second.opt_result = *msg;
  } else {
    optimization_buffer_[msg->kf_id] = *msg;
    pruneOptimizationBuffer_();
  }
  return collectUpdate_();
}

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
AssemblerUpdate KeyframeAssembler::addTags(TagArrayMsgConstSharedPtr msg)
{
  tag_buffer_.add(stampToNs(msg->header.stamp), msg, config_.buffer_duration_ns);
  return collectUpdate_();
}
#endif

AssemblerUpdate KeyframeAssembler::maintenance(int64_t now_ns)
{
  AssemblerUpdate update = collectUpdate_();

  for (auto it = pending_.begin(); it != pending_.end();) {
    if ((now_ns - it->second.created_at_ns) > config_.pending_timeout_ns) {
      update.timed_out_keyframes.push_back(makeTimedOutKeyframe_(it->second));
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }

  return update;
}

AssemblerUpdate KeyframeAssembler::collectUpdate_()
{
  AssemblerUpdate update;

  for (auto &entry : pending_) {
    resolvePending_(entry.second);
  }

  for (auto it = pending_.begin(); it != pending_.end();) {
    if (isComplete_(it->second)) {
      update.completed_records.push_back(CompletedRecord{it->second});
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }

  return update;
}

void KeyframeAssembler::resolvePending_(PendingKeyframe &pending)
{
  if (!pending.have_rgb) {
    const auto nearest = rgb_buffer_.findNearest(
      pending.keyframe_stamp_ns, config_.rgb_match_tolerance_ns);
    if (nearest.has_value()) {
      pending.have_rgb = true;
      pending.rgb_stamp_ns = nearest->first;
      pending.rgb_msg = nearest->second;
    }
  }

  if (config_.depthStreamEnabled() && !pending.have_depth) {
    const auto nearest = depth_buffer_.findNearest(
      pending.keyframe_stamp_ns, config_.depth_match_tolerance_ns);
    if (nearest.has_value()) {
      pending.have_depth = true;
      pending.depth_stamp_ns = nearest->first;
      pending.depth_msg = nearest->second;
    }
  }

#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
  if (config_.tagStreamEnabled() && !pending.have_tags) {
    const int64_t window_ns = std::max(
      config_.tag_match_tolerance_ns,
      config_.tag_aggregation_window_ns);
    auto nearby = tag_buffer_.findWithin(
      pending.keyframe_stamp_ns, window_ns);
    if (!nearby.empty()) {
      auto nearest = nearby.front();
      int64_t best_abs_dt = std::llabs(nearest.first - pending.keyframe_stamp_ns);
      for (const auto &candidate : nearby) {
        const int64_t abs_dt = std::llabs(candidate.first - pending.keyframe_stamp_ns);
        if (abs_dt < best_abs_dt) {
          best_abs_dt = abs_dt;
          nearest = candidate;
        }
      }
      pending.have_tags = true;
      pending.tags_stamp_ns = nearest.first;
      pending.tags_msg = nearest.second;
      pending.tag_window_msgs = std::move(nearby);
    }
  }
#endif
}

bool KeyframeAssembler::isComplete_(const PendingKeyframe &pending) const
{
  const bool depth_ok = !config_.require_depth || pending.have_depth;
  const bool tags_ok = !config_.require_tags || pending.have_tags;
  return pending.have_rgb && depth_ok && tags_ok && pending.have_opt_result;
}

TimedOutKeyframe KeyframeAssembler::makeTimedOutKeyframe_(const PendingKeyframe &pending) const
{
  TimedOutKeyframe timed_out;
  timed_out.kf_id = pending.keyframe.kf_id;
  timed_out.stamp_ns = pending.keyframe_stamp_ns;

  if (!pending.have_rgb) {
    timed_out.missing_fields.emplace_back("rgb");
  }
  if (config_.require_depth && !pending.have_depth) {
    timed_out.missing_fields.emplace_back("depth");
  }
  if (config_.require_tags && !pending.have_tags) {
    timed_out.missing_fields.emplace_back("tags");
  }
  if (!pending.have_opt_result) {
    timed_out.missing_fields.emplace_back("optimization_result");
  }

  return timed_out;
}

void KeyframeAssembler::pruneOptimizationBuffer_()
{
  constexpr size_t kMaxStoredOptResults = 512;
  while (optimization_buffer_.size() > kMaxStoredOptResults) {
    optimization_buffer_.erase(optimization_buffer_.begin());
  }
}

}  // namespace online_mapping_logger
