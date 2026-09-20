#pragma once

#include <cstdint>

namespace triwhirl::runtime {

constexpr std::uint64_t kRealtimeReleasePeriodUs = 1000U;
constexpr int kRealtimeReleaseInterruptPriority = 3;

// Wait for the next future hardware release. Any release that arrived while
// the current control iteration was still active is discarded so missed
// deadlines are skipped rather than replayed as catch-up iterations.
// Returns false if the GPTimer release clock could not be initialized.
bool waitForNextRealtimeRelease();

}  // namespace triwhirl::runtime
