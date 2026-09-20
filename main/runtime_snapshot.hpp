#pragma once

#include <cstdint>

namespace triwhirl::runtime {

struct RuntimeSnapshot {
  std::uint32_t t_us = 0U;

  bool attitude_initialized = false;
  bool attitude_valid = false;
  float attitude_angle_rad = 0.0F;
  float attitude_rate_rad_s = 0.0F;
  float attitude_residual_bias_rad_s = 0.0F;
  float attitude_innovation = 0.0F;
  float attitude_accel_weight = 0.0F;
  float wheel_rate_rad_s = 0.0F;

  bool safety_faulted = false;
  std::uint32_t safety_fault_mask = 0U;
  std::uint32_t safety_first_fault = 0U;

  bool telemetry_enabled = false;

  std::uint32_t timing_target_us = 0U;
  std::uint32_t timing_hard_period_us = 0U;
  std::uint64_t timing_iterations = 0U;
  std::uint32_t timing_last_exec_us = 0U;
  std::uint32_t timing_max_exec_us = 0U;
  std::uint32_t timing_min_period_us = 0U;
  std::uint32_t timing_max_period_us = 0U;
  std::uint64_t timing_overruns = 0U;
  std::uint64_t timing_late_periods = 0U;
  std::uint32_t uart_tx_drop_bytes = 0U;
};

// Single-writer (Core 1) bounded publication. The implementation keeps only
// the latest complete snapshot; publishing never blocks the realtime task.
bool initRuntimeSnapshotChannel();
void publishRuntimeSnapshot(const RuntimeSnapshot& snapshot);

// Non-blocking read for supervisory code. Returns the latest complete snapshot
// without consuming it.
bool readLatestRuntimeSnapshot(RuntimeSnapshot* snapshot);

}  // namespace triwhirl::runtime
