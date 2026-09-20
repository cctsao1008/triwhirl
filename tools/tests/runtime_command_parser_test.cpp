#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "runtime_command_parser.hpp"

namespace {

using triwhirl::runtime::RuntimeCommandParseStatus;
using triwhirl::runtime::RuntimeCommandType;
using triwhirl::runtime::parseRuntimeCommand;

void expectType(const char* text, const RuntimeCommandType type) {
  const auto parsed = parseRuntimeCommand(text);
  assert(parsed.status == RuntimeCommandParseStatus::kCommand);
  assert(parsed.command.type == type);
}

}  // namespace

int main() {
  expectType("motor stop", RuntimeCommandType::kMotorStop);
  expectType("stop", RuntimeCommandType::kStop);
  expectType("swing abort", RuntimeCommandType::kSwingAbort);
  expectType("swing start", RuntimeCommandType::kSwingStart);
  expectType("timing reset", RuntimeCommandType::kTimingReset);
  expectType("timing profile on", RuntimeCommandType::kTimingProfileOn);
  expectType("timing profile off", RuntimeCommandType::kTimingProfileOff);
  expectType("timing profile reset", RuntimeCommandType::kTimingProfileReset);
  expectType("fault clear", RuntimeCommandType::kFaultClear);
  expectType("telemetry on", RuntimeCommandType::kTelemetryOn);
  expectType("telemetry off", RuntimeCommandType::kTelemetryOff);

  {
    const auto parsed = parseRuntimeCommand("motor vq -0.625");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kMotorVq);
    assert(std::fabs(parsed.command.payload.motor_vq.volts + 0.625F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("motor config 7 -1 1.25");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kMotorConfig);
    assert(parsed.command.payload.motor_config.pole_pairs == 7);
    assert(parsed.command.payload.motor_config.sensor_direction == -1);
    assert(std::fabs(parsed.command.payload.motor_config.electrical_offset_rad - 1.25F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("motor calibrate");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kMotorCalibrate);
    assert(std::fabs(parsed.command.payload.motor_calibrate.amplitude_v - 0.6F) < 1.0e-6F);
    assert(std::fabs(parsed.command.payload.motor_calibrate.electrical_hz - 0.5F) < 1.0e-6F);
    assert(std::fabs(parsed.command.payload.motor_calibrate.turns - 4.0F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("motor calibrate 0.9 0.7 6");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kMotorCalibrate);
    assert(std::fabs(parsed.command.payload.motor_calibrate.amplitude_v - 0.9F) < 1.0e-6F);
    assert(std::fabs(parsed.command.payload.motor_calibrate.electrical_hz - 0.7F) < 1.0e-6F);
    assert(std::fabs(parsed.command.payload.motor_calibrate.turns - 6.0F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("field 3.5 0.8");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kField);
    assert(std::fabs(parsed.command.payload.field.electrical_hz - 3.5F) < 1.0e-6F);
    assert(std::fabs(parsed.command.payload.field.amplitude_v - 0.8F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("attitude reset");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kAttitudeReset);
    assert(parsed.command.payload.attitude_reset.use_accelerometer);
  }

  {
    const auto parsed = parseRuntimeCommand("attitude reset -1.25");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kAttitudeReset);
    assert(!parsed.command.payload.attitude_reset.use_accelerometer);
    assert(std::fabs(parsed.command.payload.attitude_reset.angle_rad + 1.25F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("imu calibrate");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kImuCalibrate);
    assert(parsed.command.payload.imu_calibrate.samples == 500U);
  }

  {
    const auto parsed = parseRuntimeCommand("imu calibrate 750");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kImuCalibrate);
    assert(parsed.command.payload.imu_calibrate.samples == 750U);
  }

  {
    const auto parsed = parseRuntimeCommand("imu map 0 1 2 1 -1 1");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kImuMap);
    assert(parsed.command.payload.imu_map.accel_sin_axis == 0);
    assert(parsed.command.payload.imu_map.accel_cos_axis == 1);
    assert(parsed.command.payload.imu_map.gyro_axis == 2);
    assert(parsed.command.payload.imu_map.accel_sin_sign == 1);
    assert(parsed.command.payload.imu_map.accel_cos_sign == -1);
    assert(parsed.command.payload.imu_map.gyro_sign == 1);
  }

  {
    const auto parsed = parseRuntimeCommand(
        "swing config 24 0.4 0.8 5 9 14 12.5 0.2 -1 68 20");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kSwingConfig);
    const auto& config = parsed.command.payload.swing_config;
    assert(config.target_captures == 24U);
    assert(std::fabs(config.pump_v_low - 0.4F) < 1.0e-6F);
    assert(std::fabs(config.pump_v_high - 0.8F) < 1.0e-6F);
    assert(std::fabs(config.capture_deg - 5.0F) < 1.0e-6F);
    assert(std::fabs(config.probe_exit_deg - 9.0F) < 1.0e-6F);
    assert(std::fabs(config.rearm_deg - 14.0F) < 1.0e-6F);
    assert(config.probe_duration_us == 12500U);
    assert(std::fabs(config.rate_switch_rad_s - 0.2F) < 1.0e-6F);
    assert(config.pump_polarity == -1);
    assert(std::fabs(config.vertex_a_deg - 68.0F) < 1.0e-6F);
    assert(config.max_duration_us == 20000000U);
  }

  {
    const auto parsed = parseRuntimeCommand("motor vq");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
    assert(parsed.error != nullptr);
    assert(std::strcmp(parsed.error, "ERR usage: motor vq <volts>\r\n") == 0);
  }

  {
    const auto parsed = parseRuntimeCommand("motor config 7 -1");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
    assert(parsed.error != nullptr);
  }

  {
    const auto parsed = parseRuntimeCommand("imu map 0 1 2 1 -1");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
    assert(parsed.error != nullptr);
  }

  {
    const auto parsed = parseRuntimeCommand("field 1.0");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
    assert(parsed.error != nullptr);
  }

  {
    const auto parsed = parseRuntimeCommand("swing config 24 0.4 0.8");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
    assert(parsed.error != nullptr);
  }

  {
    const auto parsed = parseRuntimeCommand("swing status");
    assert(parsed.status == RuntimeCommandParseStatus::kNotMatched);
  }

  {
    const auto parsed = parseRuntimeCommand("timing profile status");
    assert(parsed.status == RuntimeCommandParseStatus::kNotMatched);
  }

  {
    const auto parsed = parseRuntimeCommand("motor status");
    assert(parsed.status == RuntimeCommandParseStatus::kNotMatched);
  }

  return 0;
}
