#pragma once

#include "offline_global_graph/types.hpp"

namespace offline_global_graph
{

OptimizationResult optimizeSession(const SessionData &session, const OptimizerConfig &config);

}  // namespace offline_global_graph
