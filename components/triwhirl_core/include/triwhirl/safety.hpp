#pragma once

#include <cstdint>

namespace triwhirl {

enum class SafetyFault : std::uint32_t {
  kNone = 0U,
  kEncoderUnavailable = 1U << 0,
  kImuUnavailable = 1U << 1,
  kControlTiming = 1U << 2,
  kWheelOverspeed = 1U << 3,
  kBodyAngle = 1U << 4,
  kInvalidNumeric = 1U << 5,
  kBatteryUndervoltage = 1U << 6,
  kCalibration = 1U << 7,
  kStartup = 1U << 8,
  kActuator = 1U << 9,
};

constexpr std::uint32_t safetyFaultMask(const SafetyFault fault) {
  return static_cast<std::uint32_t>(fault);
}

const char* safetyFaultName(SafetyFault fault);

class SafetyLatch {
 public:
  void trip(SafetyFault fault);
  void clear();

  bool faulted() const { return mask_ != 0U; }
  std::uint32_t mask() const { return mask_; }
  SafetyFault firstFault() const { return first_fault_; }

 private:
  std::uint32_t mask_ = 0U;
  SafetyFault first_fault_ = SafetyFault::kNone;
};

}  // namespace triwhirl
