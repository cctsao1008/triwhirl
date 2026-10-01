#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>

namespace triwhirl {

struct MotorTaskTimingConfig {
  uint32_t service_period_us = 0U;
  uint32_t late_slack_us = 0U;

  bool valid() const {
    return service_period_us > 0U && service_period_us < 0x80000000U &&
           late_slack_us < 0x80000000U - service_period_us;
  }
};

struct MotorTaskTimingSnapshot {
  uint32_t service_period_us = 0U;
  uint32_t iterations = 0U;
  uint32_t last_exec_us = 0U;
  uint32_t max_exec_us = 0U;
  uint32_t min_observed_period_us = 0U;
  uint32_t max_observed_period_us = 0U;
  uint32_t late_periods = 0U;
  uint32_t deadline_misses = 0U;
  uint32_t missed_release_ticks = 0U;
  uint32_t max_notification_backlog = 0U;
  uint32_t max_command_apply_latency_us = 0U;
  bool running = false;
  bool initialization_failed = false;
  bool release_clock_failed = false;
};

namespace motor_task_timing_detail {

inline uint32_t saturatingAdd(const uint32_t value, const uint32_t increment) {
  const uint32_t remaining = UINT32_MAX - value;
  return increment > remaining ? UINT32_MAX : value + increment;
}

inline uint32_t elapsedUs32(const uint32_t now_us32,
                            const uint32_t then_us32) {
  return now_us32 - then_us32;
}

constexpr int kSnapshotReadAttempts = 3;

}  // namespace motor_task_timing_detail

// Pure timing/accounting helper. It owns no scheduler primitives and can be
// exhaustively host-tested. The target FreeRTOS wrapper feeds it actual task
// start/end timestamps and GPTimer notification counts.
class MotorTaskTimingTracker {
 public:
  explicit MotorTaskTimingTracker(const MotorTaskTimingConfig& config)
      : config_(config) {
    snapshot_.service_period_us = config.service_period_us;
  }

  bool valid() const { return config_.valid(); }

  void reset() {
    snapshot_ = {};
    snapshot_.service_period_us = config_.service_period_us;
    previous_start_us32_ = 0U;
    have_previous_start_ = false;
  }

  void recordReleaseNotifications(const uint32_t notifications,
                                  const bool one_is_current_release) {
    if (notifications == 0U) return;
    snapshot_.max_notification_backlog =
        std::max(snapshot_.max_notification_backlog, notifications);
    const uint32_t missed = one_is_current_release ? notifications - 1U
                                                   : notifications;
    snapshot_.missed_release_ticks = motor_task_timing_detail::saturatingAdd(
        snapshot_.missed_release_ticks, missed);
  }

  void recordIteration(const uint32_t start_us32, const uint32_t end_us32,
                       const uint32_t command_apply_latency_us) {
    const uint32_t exec_us =
        motor_task_timing_detail::elapsedUs32(end_us32, start_us32);
    snapshot_.iterations =
        motor_task_timing_detail::saturatingAdd(snapshot_.iterations, 1U);
    snapshot_.last_exec_us = exec_us;
    snapshot_.max_exec_us = std::max(snapshot_.max_exec_us, exec_us);
    snapshot_.max_command_apply_latency_us = std::max(
        snapshot_.max_command_apply_latency_us, command_apply_latency_us);

    if (config_.valid() && exec_us > config_.service_period_us) {
      snapshot_.deadline_misses = motor_task_timing_detail::saturatingAdd(
          snapshot_.deadline_misses, 1U);
    }

    if (have_previous_start_) {
      const uint32_t observed_period_us =
          motor_task_timing_detail::elapsedUs32(start_us32,
                                                previous_start_us32_);
      if (snapshot_.min_observed_period_us == 0U ||
          observed_period_us < snapshot_.min_observed_period_us) {
        snapshot_.min_observed_period_us = observed_period_us;
      }
      snapshot_.max_observed_period_us =
          std::max(snapshot_.max_observed_period_us, observed_period_us);
      if (config_.valid() &&
          observed_period_us > config_.service_period_us + config_.late_slack_us) {
        snapshot_.late_periods = motor_task_timing_detail::saturatingAdd(
            snapshot_.late_periods, 1U);
      }
    }

    previous_start_us32_ = start_us32;
    have_previous_start_ = true;
  }

  MotorTaskTimingSnapshot snapshot() const { return snapshot_; }

 private:
  MotorTaskTimingConfig config_{};
  MotorTaskTimingSnapshot snapshot_{};
  uint32_t previous_start_us32_ = 0U;
  bool have_previous_start_ = false;
};

// Lock-free latest-value timing publication for cross-core observability. All
// shared cells remain 32-bit so the ESP32 target does not need a hidden mutex or
// libatomic fallback.
class MotorTaskTimingMailbox {
 public:
  MotorTaskTimingMailbox() = default;
  MotorTaskTimingMailbox(const MotorTaskTimingMailbox&) = delete;
  MotorTaskTimingMailbox& operator=(const MotorTaskTimingMailbox&) = delete;

  void publish(const MotorTaskTimingSnapshot& snapshot) {
    uint32_t flags = 0U;
    if (snapshot.running) flags |= kRunningFlag;
    if (snapshot.initialization_failed) flags |= kInitializationFailedFlag;
    if (snapshot.release_clock_failed) flags |= kReleaseClockFailedFlag;

    sequence_.fetch_add(1U, std::memory_order_seq_cst);
    service_period_us_.store(snapshot.service_period_us,
                             std::memory_order_seq_cst);
    iterations_.store(snapshot.iterations, std::memory_order_seq_cst);
    last_exec_us_.store(snapshot.last_exec_us, std::memory_order_seq_cst);
    max_exec_us_.store(snapshot.max_exec_us, std::memory_order_seq_cst);
    min_period_us_.store(snapshot.min_observed_period_us,
                         std::memory_order_seq_cst);
    max_period_us_.store(snapshot.max_observed_period_us,
                         std::memory_order_seq_cst);
    late_periods_.store(snapshot.late_periods, std::memory_order_seq_cst);
    deadline_misses_.store(snapshot.deadline_misses,
                           std::memory_order_seq_cst);
    missed_release_ticks_.store(snapshot.missed_release_ticks,
                                std::memory_order_seq_cst);
    max_notification_backlog_.store(snapshot.max_notification_backlog,
                                    std::memory_order_seq_cst);
    max_command_apply_latency_us_.store(snapshot.max_command_apply_latency_us,
                                        std::memory_order_seq_cst);
    flags_.store(flags, std::memory_order_seq_cst);
    sequence_.fetch_add(1U, std::memory_order_seq_cst);
    published_.store(1U, std::memory_order_seq_cst);
  }

  bool tryRead(MotorTaskTimingSnapshot* out) const {
    if (out == nullptr || published_.load(std::memory_order_seq_cst) == 0U) {
      return false;
    }
    for (int attempt = 0;
         attempt < motor_task_timing_detail::kSnapshotReadAttempts; ++attempt) {
      const uint32_t before = sequence_.load(std::memory_order_seq_cst);
      if ((before & 1U) != 0U) continue;

      MotorTaskTimingSnapshot candidate{};
      candidate.service_period_us =
          service_period_us_.load(std::memory_order_seq_cst);
      candidate.iterations = iterations_.load(std::memory_order_seq_cst);
      candidate.last_exec_us = last_exec_us_.load(std::memory_order_seq_cst);
      candidate.max_exec_us = max_exec_us_.load(std::memory_order_seq_cst);
      candidate.min_observed_period_us =
          min_period_us_.load(std::memory_order_seq_cst);
      candidate.max_observed_period_us =
          max_period_us_.load(std::memory_order_seq_cst);
      candidate.late_periods = late_periods_.load(std::memory_order_seq_cst);
      candidate.deadline_misses =
          deadline_misses_.load(std::memory_order_seq_cst);
      candidate.missed_release_ticks =
          missed_release_ticks_.load(std::memory_order_seq_cst);
      candidate.max_notification_backlog =
          max_notification_backlog_.load(std::memory_order_seq_cst);
      candidate.max_command_apply_latency_us =
          max_command_apply_latency_us_.load(std::memory_order_seq_cst);
      const uint32_t flags = flags_.load(std::memory_order_seq_cst);
      candidate.running = (flags & kRunningFlag) != 0U;
      candidate.initialization_failed =
          (flags & kInitializationFailedFlag) != 0U;
      candidate.release_clock_failed =
          (flags & kReleaseClockFailedFlag) != 0U;

      const uint32_t after = sequence_.load(std::memory_order_seq_cst);
      if (before == after && (after & 1U) == 0U) {
        *out = candidate;
        return true;
      }
    }
    return false;
  }

 private:
  static constexpr uint32_t kRunningFlag = 1U << 0U;
  static constexpr uint32_t kInitializationFailedFlag = 1U << 1U;
  static constexpr uint32_t kReleaseClockFailedFlag = 1U << 2U;

  std::atomic<uint32_t> sequence_{0U};
  std::atomic<uint32_t> published_{0U};
  std::atomic<uint32_t> service_period_us_{0U};
  std::atomic<uint32_t> iterations_{0U};
  std::atomic<uint32_t> last_exec_us_{0U};
  std::atomic<uint32_t> max_exec_us_{0U};
  std::atomic<uint32_t> min_period_us_{0U};
  std::atomic<uint32_t> max_period_us_{0U};
  std::atomic<uint32_t> late_periods_{0U};
  std::atomic<uint32_t> deadline_misses_{0U};
  std::atomic<uint32_t> missed_release_ticks_{0U};
  std::atomic<uint32_t> max_notification_backlog_{0U};
  std::atomic<uint32_t> max_command_apply_latency_us_{0U};
  std::atomic<uint32_t> flags_{0U};
};

}  // namespace triwhirl
