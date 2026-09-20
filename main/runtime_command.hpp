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
  kMotorConfig,
  kMotorCalibrate,
  kField,
  kAttitudeReset,
  kImuCalibrate,
  kImuMap,
};

struct MotorVqPayload {
  float volts = 0.0F;
};

struct MotorConfigPayload {
  int pole_pairs = 0;
  int sensor_direction = 0;
  float electrical_offset_rad = 0.0F;
};

struct MotorCalibratePayload {
  float amplitude_v = 0.6F;
  float electrical_hz = 0.5F;
  float turns = 4.0F;
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

struct ImuMapPayload {
  int accel_sin_axis = 0;
  int accel_cos_axis = 1;
  int gyro_axis = 2;
  int accel_sin_sign = 1;
  int accel_cos_sign = 1;
  int gyro_sign = 1;
};

union RuntimeCommandPayload {
  MotorVqPayload motor_vq;
  MotorConfigPayload motor_config;
  MotorCalibratePayload motor_calibrate;
  FieldPayload field;
  AttitudeResetPayload attitude_reset;
  ImuCalibratePayload imu_calibrate;
  ImuMapPayload imu_map;

  constexpr RuntimeCommandPayload() : motor_vq{} {}
};

struct RuntimeCommand {
  RuntimeCommandType type = RuntimeCommandType::kNone;
  RuntimeCommandPayload payload{};
};

}  // namespace triwhirl::runtime
