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
  std::uint64_t drdy_edges = 0U;
  std::uint64_t drdy_consumed = 0U;
  std::uint64_t drdy_fallback_reads = 0U;
  int drdy_gpio = -1;
  bool drdy_probe_only = false;
};

// The Core-0 worker is the sole MPU FIFO consumer. When a production-board
// MPU_INT route is explicitly verified, DATA_RDY owns acquisition timing: the
// ISR wakes the worker and dispatchImuAcquisition() merely asks the coordinator
// to wait for the next IRQ-produced sample. With no verified route, the worker
// retains the request-driven FIFO fallback; Core 1 never performs MPU I2C.
bool initImuAcquisition(ImuReadFn read_fn, void* context, int core_id,
                        unsigned task_priority);
bool dispatchImuAcquisition(std::uint32_t* sequence);
bool tryCollectImuAcquisition(std::uint32_t expected_sequence,
                              ImuAcquisitionResult* result);
bool collectImuAcquisition(std::uint32_t expected_sequence,
                           std::uint32_t join_budget_us,
                           ImuAcquisitionResult* result);
ImuAcquisitionStats imuAcquisitionStats();
void resetImuAcquisitionStats();

}  // namespace triwhirl::runtime
