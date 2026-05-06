#pragma once

#include "offline_dense_map_fusion/types.hpp"

namespace offline_dense_map_fusion
{

void writeFusionOutputs(
  const SessionData &session,
  const Extrinsics &extrinsics,
  const FusionConfig &config,
  const FusionResult &result,
  const std::filesystem::path &output_dir);

}  // namespace offline_dense_map_fusion
