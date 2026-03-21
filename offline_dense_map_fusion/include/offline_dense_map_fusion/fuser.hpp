#pragma once

#include "offline_dense_map_fusion/types.hpp"

namespace offline_dense_map_fusion
{

FusionResult fuseSession(
  const SessionData &session,
  const Extrinsics &extrinsics,
  const FusionConfig &config);

}  // namespace offline_dense_map_fusion
