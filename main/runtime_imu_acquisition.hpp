#pragma once

#include <cstdint>

#include "triwhirl/drivers/mpu6050.hpp"

namespace triwhirl::runtime {

using ImuReadFn = bool (*)(void* context,
                           triwhirl::drivers::Mpu6050Sample* sample);

struct ImuAcquisitionResult {
  std::uint32_t sequence = 0U;
  std::uint32_t requested_at_us = 0U;
  std::uint32_t started_at_us = 0U;
  std::uint32_t completed_at_us = 0U;
  triwhirl::drivers::Mpu6050Sample sample{};
  bool ok = false;
};

struct ImuAcquisitionStats {
  std::uint64_t requests = 0U;
  std::uint64_t dispatch_failures = 0U;
  std::uint64_t read_failures = 0U;
  std::uint64_t stale_results = 0U;
  std::uint64_t join_timeouts = 0U;
};

// Runs the blocking MPU6050 transfer on the selected I/O core. The realtime
// caller only publishes a fixed-size request and joins a fixed-size result.
bool initImuAcquisition(ImuReadFn read_fn, void* context, int core_id,
                        unsigned task_priority);
bool dispatchImuAcquisition(std::uint32_t* sequence);
// Non-blocking result probe used by the shared Core-0 frame coordinator. It
// drains stale generations but does not count an absent result as a timeout.
bool tryCollectImuAcquisition(std::uint32_t expected_sequence,
                              ImuAcquisitionResult* result);
bool collectImuAcquisition(std::uint32_t expected_sequence,
                           std::uint32_t join_budget_us,
                           ImuAcquisitionResult* result);
ImuAcquisitionStats imuAcquisitionStats();
void resetImuAcquisitionStats();

}  // namespace triwhirl::runtime
