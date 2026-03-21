#pragma once

#include "online_mapping_logger/stamp_buffer.hpp"
#include "online_mapping_logger/types.hpp"

#include <map>
#include <unordered_map>

namespace online_mapping_logger
{

class KeyframeAssembler
{
public:
  explicit KeyframeAssembler(LoggerConfig config);

  AssemblerUpdate addRgb(ImageMsg::ConstSharedPtr msg);
  AssemblerUpdate addDepth(ImageMsg::ConstSharedPtr msg);
  AssemblerUpdate addKeyframe(KeyframeMsg::ConstSharedPtr msg, int64_t now_ns);
  AssemblerUpdate addOptimizationResult(OptimizationResultMsg::ConstSharedPtr msg);
#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
  AssemblerUpdate addTags(TagArrayMsgConstSharedPtr msg);
#endif
  AssemblerUpdate maintenance(int64_t now_ns);

private:
  AssemblerUpdate collectUpdate_();
  void resolvePending_(PendingKeyframe &pending);
  bool isComplete_(const PendingKeyframe &pending) const;
  TimedOutKeyframe makeTimedOutKeyframe_(const PendingKeyframe &pending) const;
  void pruneOptimizationBuffer_();

  LoggerConfig config_;
  StampBuffer<ImageMsg::ConstSharedPtr> rgb_buffer_;
  StampBuffer<ImageMsg::ConstSharedPtr> depth_buffer_;
#ifdef ONLINE_MAPPING_LOGGER_HAVE_APRILTAG_MSGS
  StampBuffer<TagArrayMsgConstSharedPtr> tag_buffer_;
#endif
  std::unordered_map<uint64_t, PendingKeyframe> pending_;
  std::map<uint64_t, OptimizationResultMsg> optimization_buffer_;
};

}  // namespace online_mapping_logger
