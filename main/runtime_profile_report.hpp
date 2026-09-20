#pragma once

#include <cstdint>
#include <type_traits>

namespace triwhirl::runtime {

struct RuntimeProfileReport {
  std::uint64_t requests = 0U;
  std::uint64_t completions = 0U;
  std::uint64_t dispatch_failures = 0U;
  std::uint64_t read_failures = 0U;
  std::uint64_t stale_results = 0U;
  std::uint64_t join_timeouts = 0U;
  std::uint64_t attitude_count = 0U;
  std::uint64_t attitude_total_us = 0U;
  std::uint64_t period_lt900 = 0U;
  std::uint64_t period_900_949 = 0U;
  std::uint64_t period_950_999 = 0U;
  std::uint64_t period_1000_1049 = 0U;
  std::uint64_t period_1050_1099 = 0U;
  std::uint64_t period_1100_1249 = 0U;
  std::uint64_t period_1250_1499 = 0U;
  std::uint64_t period_ge1500 = 0U;

  std::uint32_t max_consecutive_misses = 0U;
  std::uint32_t attitude_min_us = 0U;
  std::uint32_t attitude_max_us = 0U;
};

static_assert(std::is_trivially_copyable_v<RuntimeProfileReport>,
              "RuntimeProfileReport must remain trivially copyable");
static_assert(sizeof(RuntimeProfileReport) <= 152U,
              "RuntimeProfileReport grew beyond the bounded egress budget");

}  // namespace triwhirl::runtime
