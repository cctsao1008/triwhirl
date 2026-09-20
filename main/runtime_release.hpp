#pragma once

#include <cstdint>

namespace triwhirl::runtime {

constexpr std::uint64_t kRealtimeReleasePeriodUs = 1000U;
constexpr int kRealtimeReleaseInterruptPriority = 3;

struct RealtimeReleaseStats {
  std::uint64_t wait_calls = 0U;
  std::uint64_t release_ticks = 0U;
  std::uint64_t missed_release_ticks = 0U;
  std::uint64_t skip_events = 0U;
  std::uint32_t max_notification_backlog = 0U;
};

// Wait for the next future hardware release. Any release that arrived while
// the current control iteration was still active is discarded so missed
// deadlines are skipped rather than replayed as catch-up iterations.
// Returns false if the GPTimer release clock could not be initialized.
bool waitForNextRealtimeRelease();

// Scheduler counters are updated by the realtime control task itself from the
// task-notification counts returned by FreeRTOS. This avoids inferring missed
// releases from execution-time histograms. Read/reset these from the realtime
// owner (or publish a copied snapshot before crossing cores).
RealtimeReleaseStats realtimeReleaseStats();
void resetRealtimeReleaseStats();

}  // namespace triwhirl::runtime
