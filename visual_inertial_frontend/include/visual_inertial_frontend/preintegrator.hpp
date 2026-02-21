#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Core>

// Your project types (these already exist in your common/frontend types)
#include "visual_inertial_common/types.hpp"
#include "visual_inertial_frontend/types.hpp"

// Assumptions for this FIRST PASS:
// - IMU timestamps are in the SAME time domain as camera/keyframe timestamps.
// - No time offset handling.
// - You call buildAndConsume(...) only when you have IMU coverage up to t1.
// - Half-open interval: integrate (t0, t1] and then drop all samples <= t1
//   (optionally keep one anchor sample at/before t1).

struct ImuPreintegratorConfig {
  // Continuous-time noise stddevs (MVP placeholders, tune later)
  double gyro_noise_density  = 0.01;  // rad/s/sqrt(Hz)
  double accel_noise_density = 0.10;  // m/s^2/sqrt(Hz)
  double gyro_bias_rw        = 0.001; // rad/s^2/sqrt(Hz)
  double accel_bias_rw       = 0.01;  // m/s^3/sqrt(Hz)

  double gravity_mps2 = 9.81;

  // Buffer retention (seconds)
  double max_buffer_s = 5.0;

  // Keep last sample <= t1 as an anchor after consuming (recommended)
  bool keep_anchor = true;
};

class ImuPreintegrator {
public:
  using Packet    = PreintegratedImuPacket;   // from visual_inertial_common/types.hpp (or wherever you put it)

  explicit ImuPreintegrator(const ImuPreintegratorConfig& cfg = {});
  ~ImuPreintegrator() = default;

  void reset();
  void setConfig(const ImuPreintegratorConfig& cfg);
  const ImuPreintegratorConfig& config() const { return cfg_; }

  // ---- IMU ingest (thread-safe) ----
  // t_s must be in the SAME time domain as keyframe timestamps.
  void push(double t_s, const Eigen::Vector3d& accel_mps2, const Eigen::Vector3d& gyro_rps);

  // Simple buffer queries
  size_t size() const;
  bool empty() const { return size() == 0; }
  double oldestTime() const; // +inf if empty
  double newestTime() const; // -inf if empty
  bool hasCoverage(double t_s) const; // newestTime() >= t_s

  // Bias to linearize preintegration around (from backend feedback, optional for MVP)
  void setBias(const ImuBias& bias);
  ImuBias bias() const;

  // ---- Preintegration (simple API) ----
  // Build a packet for interval (t0, t1] and CONSUME samples <= t1 from the buffer.
  // Returns nullopt if there isn't enough IMU coverage.
  //
  // Packet is expected to contain:
  //  - kf_id0/kf_id1, t0/t1, bias_hat, bytes, valid flag
  std::optional<Packet> buildAndConsume(uint64_t kf_id0, double t0_s,
                                        uint64_t kf_id1, double t1_s);

private:
  void pruneLocked_();

private:
  ImuPreintegratorConfig cfg_;

  mutable std::mutex mtx_;
  std::deque<ImuSample> buf_; // must be time-ordered by t_s

  ImuBias bias_hat_;
};
