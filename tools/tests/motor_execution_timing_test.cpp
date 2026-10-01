#include <atomic>
#include <cassert>
#include <cstdint>
#include <thread>

#include "triwhirl/motor_execution_timing.hpp"

namespace {

void testConfigValidation() {
  triwhirl::MotorTaskTimingConfig config{};
  assert(!config.valid());
  config.service_period_us = 1000U;
  assert(config.valid());
  config.late_slack_us = 100U;
  assert(config.valid());
  config.service_period_us = 0x80000000U;
  assert(!config.valid());
}

void testAccounting() {
  triwhirl::MotorTaskTimingTracker tracker({1000U, 100U});
  assert(tracker.valid());

  tracker.recordReleaseNotifications(1U, true);
  tracker.recordIteration(1000U, 1100U, true, 40U);
  tracker.recordReleaseNotifications(1U, true);
  tracker.recordIteration(2000U, 2250U, true, 70U);
  tracker.recordReleaseNotifications(3U, true);
  tracker.recordIteration(3200U, 4300U, false, 999U);

  const auto snapshot = tracker.snapshot();
  assert(snapshot.service_period_us == 1000U);
  assert(snapshot.iterations == 3U);
  assert(snapshot.last_exec_us == 1100U);
  assert(snapshot.max_exec_us == 1100U);
  assert(snapshot.min_observed_period_us == 1000U);
  assert(snapshot.max_observed_period_us == 1200U);
  assert(snapshot.late_periods == 1U);
  assert(snapshot.deadline_misses == 1U);
  assert(snapshot.missed_release_ticks == 2U);
  assert(snapshot.max_notification_backlog == 3U);
  assert(snapshot.max_command_accept_latency_us == 70U);
  // First consumed command: 40 + 100 = 140 us. Second: 70 + 250 = 320 us.
  // The third iteration consumed no new command, so its 999-us fixture is ignored.
  assert(snapshot.max_command_service_complete_upper_bound_us == 320U);
}

void testReleaseBacklogAccounting() {
  triwhirl::MotorTaskTimingTracker tracker({500U, 50U});
  tracker.recordReleaseNotifications(4U, false);
  tracker.recordReleaseNotifications(3U, true);
  const auto snapshot = tracker.snapshot();
  assert(snapshot.missed_release_ticks == 6U);
  assert(snapshot.max_notification_backlog == 4U);
}

void testTimestampWraparound() {
  triwhirl::MotorTaskTimingTracker tracker({1000U, 10U});
  tracker.recordIteration(0xFFFFFF00U, 0xFFFFFF20U, true, 12U);
  tracker.recordIteration(0x000002E8U, 0x00000320U, true, 48U);
  const auto snapshot = tracker.snapshot();
  assert(snapshot.iterations == 2U);
  assert(snapshot.min_observed_period_us == 1000U);
  assert(snapshot.max_observed_period_us == 1000U);
  assert(snapshot.last_exec_us == 56U);
  assert(snapshot.max_command_accept_latency_us == 48U);
  assert(snapshot.max_command_service_complete_upper_bound_us == 104U);
  assert(snapshot.late_periods == 0U);
  assert(snapshot.deadline_misses == 0U);
}

void testSaturatingServiceCompleteUpperBound() {
  triwhirl::MotorTaskTimingTracker tracker({1000U, 0U});
  tracker.recordIteration(10U, 30U, true, UINT32_MAX - 10U);
  const auto snapshot = tracker.snapshot();
  assert(snapshot.max_command_accept_latency_us == UINT32_MAX - 10U);
  assert(snapshot.max_command_service_complete_upper_bound_us == UINT32_MAX);
}

void testReset() {
  triwhirl::MotorTaskTimingTracker tracker({1000U, 100U});
  tracker.recordIteration(100U, 150U, true, 20U);
  tracker.reset();
  const auto snapshot = tracker.snapshot();
  assert(snapshot.service_period_us == 1000U);
  assert(snapshot.iterations == 0U);
  assert(snapshot.max_exec_us == 0U);
  assert(snapshot.min_observed_period_us == 0U);
  assert(snapshot.max_command_accept_latency_us == 0U);
  assert(snapshot.max_command_service_complete_upper_bound_us == 0U);
}

void testTimingMailboxBasics() {
  triwhirl::MotorTaskTimingMailbox mailbox;
  triwhirl::MotorTaskTimingSnapshot snapshot{};
  assert(!mailbox.tryRead(&snapshot));

  triwhirl::MotorTaskTimingSnapshot written{};
  written.service_period_us = 500U;
  written.iterations = 7U;
  written.last_exec_us = 80U;
  written.max_exec_us = 120U;
  written.min_observed_period_us = 490U;
  written.max_observed_period_us = 530U;
  written.late_periods = 2U;
  written.deadline_misses = 1U;
  written.missed_release_ticks = 3U;
  written.max_notification_backlog = 4U;
  written.max_command_accept_latency_us = 150U;
  written.max_command_service_complete_upper_bound_us = 230U;
  written.running = true;
  written.initialization_failed = false;
  written.release_clock_failed = true;
  mailbox.publish(written);

  assert(mailbox.tryRead(&snapshot));
  assert(snapshot.service_period_us == 500U);
  assert(snapshot.iterations == 7U);
  assert(snapshot.last_exec_us == 80U);
  assert(snapshot.max_exec_us == 120U);
  assert(snapshot.min_observed_period_us == 490U);
  assert(snapshot.max_observed_period_us == 530U);
  assert(snapshot.late_periods == 2U);
  assert(snapshot.deadline_misses == 1U);
  assert(snapshot.missed_release_ticks == 3U);
  assert(snapshot.max_notification_backlog == 4U);
  assert(snapshot.max_command_accept_latency_us == 150U);
  assert(snapshot.max_command_service_complete_upper_bound_us == 230U);
  assert(snapshot.running);
  assert(!snapshot.initialization_failed);
  assert(snapshot.release_clock_failed);
}

void testTimingMailboxConcurrentCoherence() {
  constexpr std::uint32_t kCount = 100000U;
  triwhirl::MotorTaskTimingMailbox mailbox;
  std::atomic<bool> writer_done{false};

  std::thread writer([&]() {
    for (std::uint32_t i = 1U; i <= kCount; ++i) {
      triwhirl::MotorTaskTimingSnapshot snapshot{};
      snapshot.service_period_us = 500U;
      snapshot.iterations = i;
      snapshot.last_exec_us = i ^ 0x11110000U;
      snapshot.max_exec_us = i ^ 0x22220000U;
      snapshot.min_observed_period_us = i ^ 0x33330000U;
      snapshot.max_observed_period_us = i ^ 0x44440000U;
      snapshot.late_periods = i * 2U;
      snapshot.deadline_misses = i * 3U;
      snapshot.missed_release_ticks = i * 4U;
      snapshot.max_notification_backlog = i * 5U;
      snapshot.max_command_accept_latency_us = i * 6U;
      snapshot.max_command_service_complete_upper_bound_us = i * 7U;
      snapshot.running = (i & 1U) != 0U;
      snapshot.initialization_failed = (i & 2U) != 0U;
      snapshot.release_clock_failed = (i & 4U) != 0U;
      mailbox.publish(snapshot);
    }
    writer_done.store(true, std::memory_order_release);
  });

  std::uint32_t last = 0U;
  while (!writer_done.load(std::memory_order_acquire) || last < kCount) {
    triwhirl::MotorTaskTimingSnapshot snapshot{};
    if (!mailbox.tryRead(&snapshot)) {
      std::this_thread::yield();
      continue;
    }
    const std::uint32_t i = snapshot.iterations;
    assert(snapshot.service_period_us == 500U);
    assert(snapshot.last_exec_us == (i ^ 0x11110000U));
    assert(snapshot.max_exec_us == (i ^ 0x22220000U));
    assert(snapshot.min_observed_period_us == (i ^ 0x33330000U));
    assert(snapshot.max_observed_period_us == (i ^ 0x44440000U));
    assert(snapshot.late_periods == i * 2U);
    assert(snapshot.deadline_misses == i * 3U);
    assert(snapshot.missed_release_ticks == i * 4U);
    assert(snapshot.max_notification_backlog == i * 5U);
    assert(snapshot.max_command_accept_latency_us == i * 6U);
    assert(snapshot.max_command_service_complete_upper_bound_us == i * 7U);
    assert(snapshot.running == ((i & 1U) != 0U));
    assert(snapshot.initialization_failed == ((i & 2U) != 0U));
    assert(snapshot.release_clock_failed == ((i & 4U) != 0U));
    assert(i >= last);
    last = i;
  }

  writer.join();
  assert(last == kCount);
}

}  // namespace

int main() {
  testConfigValidation();
  testAccounting();
  testReleaseBacklogAccounting();
  testTimestampWraparound();
  testSaturatingServiceCompleteUpperBound();
  testReset();
  testTimingMailboxBasics();
  testTimingMailboxConcurrentCoherence();
  return 0;
}
