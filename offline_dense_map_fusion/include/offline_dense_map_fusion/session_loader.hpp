#pragma once

#include "offline_dense_map_fusion/types.hpp"

namespace offline_dense_map_fusion
{

SessionData loadSessionWithOptimizedPoses(const std::filesystem::path &session_dir);
Extrinsics loadExtrinsics(const std::filesystem::path &path);

}  // namespace offline_dense_map_fusion
