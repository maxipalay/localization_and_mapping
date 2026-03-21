#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace online_mapping_logger
{

inline std::string nowUtcString()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &now_c);
#else
  gmtime_r(&now_c, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

inline std::string defaultSessionName()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &now_c);
#else
  localtime_r(&now_c, &tm);
#endif
  std::ostringstream oss;
  oss << "session_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

inline std::string sanitizeFilenameComponent(const std::string &input)
{
  std::string out;
  out.reserve(input.size());
  for (const char c : input)
  {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')
    {
      out.push_back(c);
    }
    else
    {
      out.push_back('_');
    }
  }
  return out.empty() ? std::string("session") : out;
}

inline std::string formatKfId(uint64_t kf_id)
{
  std::ostringstream oss;
  oss << "kf_" << std::setw(6) << std::setfill('0') << kf_id;
  return oss.str();
}

inline std::string toHex(const std::vector<uint8_t> &bytes)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (const uint8_t byte : bytes)
  {
    oss << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

}  // namespace online_mapping_logger
