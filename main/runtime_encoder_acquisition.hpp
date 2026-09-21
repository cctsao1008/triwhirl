#pragma once

#include <cstdint>

namespace triwhirl::runtime {

using EncoderReadFn = bool (*)(void* context, std::uint16_t* raw_count);

struct EncoderAcquisitionResult {
  std::uint32_t sequence = 0U;
  std::uint32_t requested_at_us = 0U;
  std::uint32_t started_at_us = 0U;
  std::uint32_t completed_at_us = 0U;
  std::uint16_t raw_count = 0U;
  bool ok = false;
};

struct EncoderAcquisitionStats {
  std::uint64_t requests = 0U;
  std::uint64_t dispatch_failures = 0U;
  std::uint64_t read_failures = 0U;
  std::uint64_t stale_results = 0U;
  std::uint64_t join_timeouts = 0U;
};

bool initEncoderAcquisition(EncoderReadFn read_fn, void* context, int core_id,
                            unsigned task_priority);
bool dispatchEncoderAcquisition(std::uint32_t* sequence);
bool collectEncoderAcquisition(std::uint32_t expected_sequence,
                               std::uint32_t join_budget_us,
                               EncoderAcquisitionResult* result);
EncoderAcquisitionStats encoderAcquisitionStats();
void resetEncoderAcquisitionStats();

}  // namespace triwhirl::runtime
