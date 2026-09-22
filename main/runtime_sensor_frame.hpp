#pragma once

#include <cstdint>
#include <type_traits>

#include "runtime_encoder_acquisition.hpp"
#include "runtime_imu_acquisition.hpp"

namespace triwhirl::runtime {

// One request generation binds the AS5600 and MPU6050 acquisitions together.
// Individual sensor validity is preserved so bring-up modes can degrade
// independently. Closed-loop Balance requires complete=true, so its state
// feedback never mixes independently fresh members from different generations.
struct RuntimeSensorFrame {
  std::uint32_t sequence = 0U;
  std::uint32_t requested_at_us = 0U;
  std::uint32_t published_at_us = 0U;
  EncoderAcquisitionResult encoder{};
  ImuAcquisitionResult imu{};
  bool encoder_received = false;
  bool imu_expected = false;
  bool imu_received = false;
  bool complete = false;
};

static_assert(std::is_trivially_copyable_v<RuntimeSensorFrame>,
              "RuntimeSensorFrame must remain trivially copyable");
static_assert(sizeof(RuntimeSensorFrame) <= 128U,
              "RuntimeSensorFrame grew beyond the bounded mailbox budget");

// Runtime bring-up modes may ride through one short producer/scheduler phase
// slip without reclassifying a healthy sensor as unavailable. The nominal
// freshness budget remains 3 ms at the caller; this extra 3 ms is a bounded
// hard-availability grace, not a new Balance feedback requirement.
inline constexpr std::uint32_t kSensorTransientFreshnessGraceUs = 3000U;

// Unsigned subtraction is wrap-safe when `now_us` is chronologically after the
// sample. A frame can also be published concurrently after Core 1 captured its
// loop timestamp; in that case the raw subtraction lands in the upper half of
// the uint32 range. Treat that short future skew as zero age rather than a stale
// sample. Real sensor ages are orders of magnitude below the half-range (~35 min).
constexpr std::uint32_t sensorTimestampAgeUs(const std::uint32_t now_us,
                                             const std::uint32_t timestamp_us) {
  const std::uint32_t age = now_us - timestamp_us;
  return age <= 0x7FFFFFFFU ? age : 0U;
}

// Exact freshness predicate used by Balance and any other state-feedback path
// that must reject samples as soon as the nominal age budget is exceeded.
constexpr bool sensorTimestampNominallyFresh(
    const std::uint32_t now_us, const std::uint32_t timestamp_us,
    const std::uint32_t max_age_us) {
  return timestamp_us != 0U &&
         sensorTimestampAgeUs(now_us, timestamp_us) <= max_age_us;
}

// Operational availability predicate used by the generic runtime freshness
// gate. A healthy acquisition pipeline is allowed one bounded transient grace
// window before encoder/IMU availability is revoked. Balance separately applies
// sensorTimestampNominallyFresh() to the exact consumed generation.
constexpr bool sensorTimestampFresh(const std::uint32_t now_us,
                                    const std::uint32_t timestamp_us,
                                    const std::uint32_t max_age_us) {
  if (timestamp_us == 0U) {
    return false;
  }
  const std::uint64_t hard_limit_us =
      static_cast<std::uint64_t>(max_age_us) +
      static_cast<std::uint64_t>(kSensorTransientFreshnessGraceUs);
  return static_cast<std::uint64_t>(
             sensorTimestampAgeUs(now_us, timestamp_us)) <= hard_limit_us;
}

constexpr std::uint32_t encoderSampleTimestampUs(
    const EncoderAcquisitionResult& result) {
  return result.completed_at_us != 0U ? result.completed_at_us
                                      : result.requested_at_us;
}

constexpr std::uint32_t imuSampleTimestampUs(
    const ImuAcquisitionResult& result) {
  return result.completed_at_us != 0U ? result.completed_at_us
                                      : result.requested_at_us;
}

}  // namespace triwhirl::runtime
