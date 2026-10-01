#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace triwhirl {

// The motor/attitude boundary is intended to cross ESP32 task/core domains
// without a mutex, queue backlog, or plain shared non-atomic structs.  Keep
// every shared cell 32-bit so the contract can require native lock-free atomic
// access on the target rather than silently falling back to a library lock.
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "Motor mailboxes require lock-free 32-bit atomics");
static_assert(sizeof(float) == sizeof(uint32_t),
              "Motor mailbox float transport requires 32-bit float");

namespace motor_mailbox_detail {

inline uint32_t floatToBits(const float value) {
  uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

inline float bitsToFloat(const uint32_t bits) {
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// A deliberately small fixed retry budget keeps control-domain reads bounded.
// Failure means "use the previous accepted snapshot", never "spin until fresh".
constexpr int kSnapshotReadAttempts = 3;

}  // namespace motor_mailbox_detail

struct MotorCommandSnapshot {
  float target_velocity_rad_s = 0.0F;
  uint32_t generation = 0U;
  uint32_t issued_at_us32 = 0U;
  bool enabled = false;
};

struct MotorObservationSnapshot {
  float shaft_velocity_rad_s = 0.0F;
  float applied_target_velocity_rad_s = 0.0F;
  uint32_t observation_generation = 0U;
  uint32_t applied_command_generation = 0U;
  uint32_t serviced_at_us32 = 0U;
  bool initialized = false;
  bool sensor_valid = false;
  bool backend_faulted = false;
  bool actuator_enabled = false;
};

// Single-producer / single-consumer, latest-value command mailbox.
//
// The attitude domain is the only writer. The motor domain is the only reader.
// publish*() overwrites stale commands rather than building a queue, which is
// the desired semantic for a realtime velocity target.  generation is assigned
// by the mailbox and skips zero so zero can remain "not yet commanded".
class MotorCommandMailbox {
 public:
  MotorCommandMailbox() = default;
  MotorCommandMailbox(const MotorCommandMailbox&) = delete;
  MotorCommandMailbox& operator=(const MotorCommandMailbox&) = delete;

  uint32_t publishTarget(const float target_velocity_rad_s,
                         const uint32_t issued_at_us32) {
    if (!std::isfinite(target_velocity_rad_s)) {
      return 0U;
    }
    return publishImpl(target_velocity_rad_s, true, issued_at_us32);
  }

  uint32_t publishStop(const uint32_t issued_at_us32) {
    return publishImpl(0.0F, false, issued_at_us32);
  }

  bool tryRead(MotorCommandSnapshot* out) const {
    if (out == nullptr ||
        published_.load(std::memory_order_seq_cst) == 0U) {
      return false;
    }

    for (int attempt = 0;
         attempt < motor_mailbox_detail::kSnapshotReadAttempts; ++attempt) {
      const uint32_t before = sequence_.load(std::memory_order_seq_cst);
      if ((before & 1U) != 0U) {
        continue;
      }

      MotorCommandSnapshot candidate{};
      candidate.target_velocity_rad_s = motor_mailbox_detail::bitsToFloat(
          target_bits_.load(std::memory_order_seq_cst));
      candidate.generation = generation_.load(std::memory_order_seq_cst);
      candidate.issued_at_us32 = issued_at_us32_.load(std::memory_order_seq_cst);
      candidate.enabled =
          (flags_.load(std::memory_order_seq_cst) & kEnabledFlag) != 0U;

      const uint32_t after = sequence_.load(std::memory_order_seq_cst);
      if (before == after && (after & 1U) == 0U) {
        *out = candidate;
        return true;
      }
    }
    return false;
  }

 private:
  static constexpr uint32_t kEnabledFlag = 1U << 0U;

  uint32_t nextGeneration() {
    ++writer_generation_;
    if (writer_generation_ == 0U) {
      ++writer_generation_;
    }
    return writer_generation_;
  }

  uint32_t publishImpl(const float target_velocity_rad_s, const bool enabled,
                       const uint32_t issued_at_us32) {
    const uint32_t generation = nextGeneration();

    // Sequence-counter protocol over atomic payload cells.  seq_cst is
    // intentional here: this transport values a simple, auditable ordering
    // contract over shaving a few cycles from a once-per-control-tick mailbox.
    sequence_.fetch_add(1U, std::memory_order_seq_cst);  // odd: write in flight
    target_bits_.store(motor_mailbox_detail::floatToBits(target_velocity_rad_s),
                       std::memory_order_seq_cst);
    generation_.store(generation, std::memory_order_seq_cst);
    issued_at_us32_.store(issued_at_us32, std::memory_order_seq_cst);
    flags_.store(enabled ? kEnabledFlag : 0U, std::memory_order_seq_cst);
    sequence_.fetch_add(1U, std::memory_order_seq_cst);  // even: coherent
    published_.store(1U, std::memory_order_seq_cst);
    return generation;
  }

  // writer_generation_ is writer-domain-only state and is therefore not shared.
  uint32_t writer_generation_ = 0U;
  std::atomic<uint32_t> sequence_{0U};
  std::atomic<uint32_t> published_{0U};
  std::atomic<uint32_t> target_bits_{0U};
  std::atomic<uint32_t> generation_{0U};
  std::atomic<uint32_t> issued_at_us32_{0U};
  std::atomic<uint32_t> flags_{0U};
};

// Single-producer / single-consumer, latest-value observation mailbox.
//
// The motor domain is the only writer. The attitude domain is the only reader.
// applied_command_generation links a published motor observation to the command
// the backend actually consumed, which later allows command-to-apply latency to
// be measured without exposing motor-library state.
class MotorObservationMailbox {
 public:
  MotorObservationMailbox() = default;
  MotorObservationMailbox(const MotorObservationMailbox&) = delete;
  MotorObservationMailbox& operator=(const MotorObservationMailbox&) = delete;

  uint32_t publish(const float shaft_velocity_rad_s,
                   const float applied_target_velocity_rad_s,
                   const uint32_t applied_command_generation,
                   const uint32_t serviced_at_us32, const bool initialized,
                   const bool sensor_valid, const bool backend_faulted,
                   const bool actuator_enabled) {
    if (!std::isfinite(shaft_velocity_rad_s) ||
        !std::isfinite(applied_target_velocity_rad_s)) {
      return 0U;
    }

    const uint32_t observation_generation = nextGeneration();
    uint32_t flags = 0U;
    if (initialized) {
      flags |= kInitializedFlag;
    }
    if (sensor_valid) {
      flags |= kSensorValidFlag;
    }
    if (backend_faulted) {
      flags |= kBackendFaultedFlag;
    }
    if (actuator_enabled) {
      flags |= kActuatorEnabledFlag;
    }

    sequence_.fetch_add(1U, std::memory_order_seq_cst);  // odd
    shaft_velocity_bits_.store(
        motor_mailbox_detail::floatToBits(shaft_velocity_rad_s),
        std::memory_order_seq_cst);
    applied_target_bits_.store(
        motor_mailbox_detail::floatToBits(applied_target_velocity_rad_s),
        std::memory_order_seq_cst);
    observation_generation_.store(observation_generation,
                                  std::memory_order_seq_cst);
    applied_command_generation_.store(applied_command_generation,
                                      std::memory_order_seq_cst);
    serviced_at_us32_.store(serviced_at_us32, std::memory_order_seq_cst);
    flags_.store(flags, std::memory_order_seq_cst);
    sequence_.fetch_add(1U, std::memory_order_seq_cst);  // even
    published_.store(1U, std::memory_order_seq_cst);
    return observation_generation;
  }

  bool tryRead(MotorObservationSnapshot* out) const {
    if (out == nullptr ||
        published_.load(std::memory_order_seq_cst) == 0U) {
      return false;
    }

    for (int attempt = 0;
         attempt < motor_mailbox_detail::kSnapshotReadAttempts; ++attempt) {
      const uint32_t before = sequence_.load(std::memory_order_seq_cst);
      if ((before & 1U) != 0U) {
        continue;
      }

      MotorObservationSnapshot candidate{};
      candidate.shaft_velocity_rad_s = motor_mailbox_detail::bitsToFloat(
          shaft_velocity_bits_.load(std::memory_order_seq_cst));
      candidate.applied_target_velocity_rad_s =
          motor_mailbox_detail::bitsToFloat(
              applied_target_bits_.load(std::memory_order_seq_cst));
      candidate.observation_generation =
          observation_generation_.load(std::memory_order_seq_cst);
      candidate.applied_command_generation =
          applied_command_generation_.load(std::memory_order_seq_cst);
      candidate.serviced_at_us32 =
          serviced_at_us32_.load(std::memory_order_seq_cst);
      const uint32_t flags = flags_.load(std::memory_order_seq_cst);
      candidate.initialized = (flags & kInitializedFlag) != 0U;
      candidate.sensor_valid = (flags & kSensorValidFlag) != 0U;
      candidate.backend_faulted = (flags & kBackendFaultedFlag) != 0U;
      candidate.actuator_enabled = (flags & kActuatorEnabledFlag) != 0U;

      const uint32_t after = sequence_.load(std::memory_order_seq_cst);
      if (before == after && (after & 1U) == 0U) {
        *out = candidate;
        return true;
      }
    }
    return false;
  }

 private:
  static constexpr uint32_t kInitializedFlag = 1U << 0U;
  static constexpr uint32_t kSensorValidFlag = 1U << 1U;
  static constexpr uint32_t kBackendFaultedFlag = 1U << 2U;
  static constexpr uint32_t kActuatorEnabledFlag = 1U << 3U;

  uint32_t nextGeneration() {
    ++writer_generation_;
    if (writer_generation_ == 0U) {
      ++writer_generation_;
    }
    return writer_generation_;
  }

  uint32_t writer_generation_ = 0U;
  std::atomic<uint32_t> sequence_{0U};
  std::atomic<uint32_t> published_{0U};
  std::atomic<uint32_t> shaft_velocity_bits_{0U};
  std::atomic<uint32_t> applied_target_bits_{0U};
  std::atomic<uint32_t> observation_generation_{0U};
  std::atomic<uint32_t> applied_command_generation_{0U};
  std::atomic<uint32_t> serviced_at_us32_{0U};
  std::atomic<uint32_t> flags_{0U};
};

}  // namespace triwhirl
