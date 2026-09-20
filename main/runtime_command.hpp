#pragma once

#include <cstdint>

namespace triwhirl::runtime {

// Commands that have crossed the supervisor/realtime ownership boundary.
// UART development CLI and BLE GATT ingress are parsed/validated in the
// supervisor domain; realtime receives only this fixed-size record.
enum class RuntimeCommandType : std::uint8_t {
  kNone = 0,
  kMotorStop,
  kStop,
  kSwingAbort,
  kTimingReset,
  kFaultClear,
  kTelemetryOn,
  kTelemetryOff,
  kMotorVq,
  kField,
  kAttitudeReset,
  kImuCalibrate,
};

struct MotorVqPayload {
  float volts = 0.0F;
};

struct FieldPayload {
  float electrical_hz = 0.0F;
  float amplitude_v = 0.0F;
};

struct AttitudeResetPayload {
  float angle_rad = 0.0F;
  bool use_accelerometer = true;
};

struct ImuCalibratePayload {
  std::uint32_t samples = 0U;
};

union RuntimeCommandPayload {
  MotorVqPayload motor_vq;
  FieldPayload field;
  AttitudeResetPayload attitude_reset;
  ImuCalibratePayload imu_calibrate;

  constexpr RuntimeCommandPayload() : motor_vq{} {}
};

struct RuntimeCommand {
  RuntimeCommandType type = RuntimeCommandType::kNone;
  RuntimeCommandPayload payload{};
};

}  // namespace triwhirl::runtime
