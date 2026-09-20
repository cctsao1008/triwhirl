#pragma once

#include <cstdint>
#include <type_traits>

namespace triwhirl::runtime {

// UART development CLI and BLE GATT ingress are parsed/validated in the
// supervisor domain. Realtime receives only this fixed-size typed record.
// Snapshot/transport-owned read-only commands are handled entirely on Core 0
// and therefore are intentionally absent from this enum.
enum class RuntimeCommandType : std::uint8_t {
  kNone = 0,
  kSwingStatus,
  kTimingProfileStatus,
  kMotorStop,
  kStop,
  kSwingAbort,
  kSwingStart,
  kSwingConfig,
  kTimingReset,
  kTimingProfileOn,
  kTimingProfileOff,
  kTimingProfileReset,
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
  kLogPrepare,
  kLogStart,
  kLogCriticalOn,
  kLogCriticalOff,
  kLogStop,
  kLogDump,
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

struct SwingConfigPayload {
  std::uint32_t target_captures = 0U;
  float pump_v_low = 0.0F;
  float pump_v_high = 0.0F;
  float capture_deg = 0.0F;
  float probe_exit_deg = 0.0F;
  float rearm_deg = 0.0F;
  std::uint32_t probe_duration_us = 0U;
  float rate_switch_rad_s = 0.0F;
  int pump_polarity = 0;
  float vertex_a_deg = 0.0F;
  std::uint32_t max_duration_us = 0U;
};

struct LogPreparePayload {
  float seconds = 0.0F;
};

union RuntimeCommandPayload {
  MotorVqPayload motor_vq;
  MotorConfigPayload motor_config;
  MotorCalibratePayload motor_calibrate;
  FieldPayload field;
  AttitudeResetPayload attitude_reset;
  ImuCalibratePayload imu_calibrate;
  ImuMapPayload imu_map;
  SwingConfigPayload swing_config;
  LogPreparePayload log_prepare;

  constexpr RuntimeCommandPayload() : motor_vq{} {}
};

struct RuntimeCommand {
  RuntimeCommandType type = RuntimeCommandType::kNone;
  RuntimeCommandPayload payload{};
};

// FreeRTOS queues copy records by value. Keep this boundary POD-like and small
// enough that command publication remains deterministic and bounded.
static_assert(std::is_trivially_copyable_v<RuntimeCommand>,
              "RuntimeCommand must remain trivially copyable");
static_assert(sizeof(RuntimeCommand) <= 64U,
              "RuntimeCommand grew beyond the bounded mailbox budget");

}  // namespace triwhirl::runtime
