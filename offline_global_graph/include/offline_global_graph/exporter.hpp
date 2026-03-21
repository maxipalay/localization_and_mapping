#pragma once

#include "offline_global_graph/types.hpp"

#include <filesystem>

namespace offline_global_graph
{

void writeOptimizationOutputs(
  const SessionData &session,
  const OptimizationResult &result,
  const std::filesystem::path &output_dir);

}  // namespace offline_global_graph
