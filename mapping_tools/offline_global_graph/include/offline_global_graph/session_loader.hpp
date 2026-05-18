#pragma once

#include "offline_global_graph/types.hpp"

#include <filesystem>

namespace offline_global_graph
{

SessionData loadSession(const std::filesystem::path &session_dir);

}  // namespace offline_global_graph
