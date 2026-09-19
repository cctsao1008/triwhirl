#include "triwhirl/safety.hpp"

namespace triwhirl {

const char* safetyFaultName(const SafetyFault fault) {
  switch (fault) {
    case SafetyFault::kNone:
      return "none";
    case SafetyFault::kEncoderUnavailable:
      return "encoder_unavailable";
    case SafetyFault::kImuUnavailable:
      return "imu_unavailable";
    case SafetyFault::kControlTiming:
      return "control_timing";
    case SafetyFault::kWheelOverspeed:
      return "wheel_overspeed";
    case SafetyFault::kBodyAngle:
      return "body_angle";
    case SafetyFault::kInvalidNumeric:
      return "invalid_numeric";
    case SafetyFault::kBatteryUndervoltage:
      return "battery_undervoltage";
    case SafetyFault::kCalibration:
      return "calibration";
    case SafetyFault::kStartup:
      return "startup";
    case SafetyFault::kActuator:
      return "actuator";
  }
  return "unknown";
}

void SafetyLatch::trip(const SafetyFault fault) {
  if (fault == SafetyFault::kNone) {
    return;
  }
  const std::uint32_t bit = safetyFaultMask(fault);
  if (mask_ == 0U) {
    first_fault_ = fault;
  }
  mask_ |= bit;
}

void SafetyLatch::clear() {
  mask_ = 0U;
  first_fault_ = SafetyFault::kNone;
}

}  // namespace triwhirl
