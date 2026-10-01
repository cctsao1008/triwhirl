#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>

#include "triwhirl/motor_mailbox.hpp"

namespace {

void testCommandBasics() {
  triwhirl::MotorCommandMailbox mailbox;
  triwhirl::MotorCommandSnapshot snapshot{};

  assert(!mailbox.tryRead(&snapshot));
  assert(!mailbox.tryRead(nullptr));

  const uint32_t generation = mailbox.publishTarget(12.5F, 1234U);
  assert(generation == 1U);
  assert(mailbox.tryRead(&snapshot));
  assert(snapshot.enabled);
  assert(snapshot.generation == 1U);
  assert(snapshot.issued_at_us32 == 1234U);
  assert(snapshot.target_velocity_rad_s == 12.5F);

  // Invalid commands are rejected without consuming a generation.
  assert(mailbox.publishTarget(std::numeric_limits<float>::quiet_NaN(), 2U) ==
         0U);
  assert(mailbox.publishTarget(std::numeric_limits<float>::infinity(), 2U) ==
         0U);

  const uint32_t stop_generation = mailbox.publishStop(1300U);
  assert(stop_generation == 2U);
  assert(mailbox.tryRead(&snapshot));
  assert(!snapshot.enabled);
  assert(snapshot.generation == 2U);
  assert(snapshot.issued_at_us32 == 1300U);
  assert(snapshot.target_velocity_rad_s == 0.0F);

  // Latest-value semantics: stale targets are overwritten rather than queued.
  assert(mailbox.publishTarget(-5.0F, 1400U) == 3U);
  assert(mailbox.publishTarget(7.0F, 1500U) == 4U);
  assert(mailbox.tryRead(&snapshot));
  assert(snapshot.generation == 4U);
  assert(snapshot.target_velocity_rad_s == 7.0F);
  assert(snapshot.issued_at_us32 == 1500U);
}

void testObservationBasics() {
  triwhirl::MotorObservationMailbox mailbox;
  triwhirl::MotorObservationSnapshot snapshot{};

  assert(!mailbox.tryRead(&snapshot));
  assert(mailbox.publish(std::numeric_limits<float>::quiet_NaN(), 0.0F, 1U,
                         9U, 10U, true, false, true, false, false) == 0U);

  const uint32_t generation = mailbox.publish(
      3.5F, -2.0F, 17U, 23U, 900U, true, true, false, true, true);
  assert(generation == 1U);
  assert(mailbox.tryRead(&snapshot));
  assert(snapshot.observation_generation == 1U);
  assert(snapshot.applied_command_generation == 17U);
  assert(snapshot.command_apply_latency_us == 23U);
  assert(snapshot.serviced_at_us32 == 900U);
  assert(snapshot.shaft_velocity_rad_s == 3.5F);
  assert(snapshot.applied_target_velocity_rad_s == -2.0F);
  assert(snapshot.initialized);
  assert(snapshot.sensor_valid);
  assert(!snapshot.backend_faulted);
  assert(snapshot.actuator_enabled);
  assert(snapshot.command_timed_out);
}

void testCommandConcurrentCoherence() {
  constexpr uint32_t kCount = 100000U;
  triwhirl::MotorCommandMailbox mailbox;
  std::atomic<bool> writer_done{false};

  std::thread writer([&]() {
    for (uint32_t i = 1U; i <= kCount; ++i) {
      const float target = static_cast<float>(i) * 0.25F;
      const uint32_t generation =
          mailbox.publishTarget(target, i ^ 0xA5A50000U);
      assert(generation == i);
    }
    writer_done.store(true, std::memory_order_release);
  });

  uint32_t last_generation = 0U;
  while (!writer_done.load(std::memory_order_acquire) ||
         last_generation < kCount) {
    triwhirl::MotorCommandSnapshot snapshot{};
    if (!mailbox.tryRead(&snapshot)) {
      std::this_thread::yield();
      continue;
    }

    // If a tuple were torn, at least one of these independent relationships
    // would almost certainly refer to a different publication generation.
    assert(snapshot.enabled);
    assert(snapshot.target_velocity_rad_s * 4.0F ==
           static_cast<float>(snapshot.generation));
    assert(snapshot.issued_at_us32 ==
           (snapshot.generation ^ 0xA5A50000U));
    assert(snapshot.generation >= last_generation);
    last_generation = snapshot.generation;
  }

  writer.join();
  assert(last_generation == kCount);
}

void testObservationConcurrentCoherence() {
  constexpr uint32_t kCount = 100000U;
  triwhirl::MotorObservationMailbox mailbox;
  std::atomic<bool> writer_done{false};

  std::thread writer([&]() {
    for (uint32_t i = 1U; i <= kCount; ++i) {
      const uint32_t observation_generation = mailbox.publish(
          static_cast<float>(i) * 0.5F, -static_cast<float>(i) * 0.25F,
          i * 3U, i * 7U, i ^ 0x5A5A0000U, (i & 1U) != 0U,
          (i & 2U) != 0U, (i & 4U) != 0U, (i & 8U) != 0U,
          (i & 16U) != 0U);
      assert(observation_generation == i);
    }
    writer_done.store(true, std::memory_order_release);
  });

  uint32_t last_generation = 0U;
  while (!writer_done.load(std::memory_order_acquire) ||
         last_generation < kCount) {
    triwhirl::MotorObservationSnapshot snapshot{};
    if (!mailbox.tryRead(&snapshot)) {
      std::this_thread::yield();
      continue;
    }

    const uint32_t generation = snapshot.observation_generation;
    assert(snapshot.shaft_velocity_rad_s * 2.0F ==
           static_cast<float>(generation));
    assert(snapshot.applied_target_velocity_rad_s * -4.0F ==
           static_cast<float>(generation));
    assert(snapshot.applied_command_generation == generation * 3U);
    assert(snapshot.command_apply_latency_us == generation * 7U);
    assert(snapshot.serviced_at_us32 == (generation ^ 0x5A5A0000U));
    assert(snapshot.initialized == ((generation & 1U) != 0U));
    assert(snapshot.sensor_valid == ((generation & 2U) != 0U));
    assert(snapshot.backend_faulted == ((generation & 4U) != 0U));
    assert(snapshot.actuator_enabled == ((generation & 8U) != 0U));
    assert(snapshot.command_timed_out == ((generation & 16U) != 0U));
    assert(generation >= last_generation);
    last_generation = generation;
  }

  writer.join();
  assert(last_generation == kCount);
}

}  // namespace

int main() {
  testCommandBasics();
  testObservationBasics();
  testCommandConcurrentCoherence();
  testObservationConcurrentCoherence();
  return 0;
}
