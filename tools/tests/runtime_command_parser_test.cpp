#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "runtime_command_parser.hpp"

namespace {

using triwhirl::runtime::RuntimeCommandParseStatus;
using triwhirl::runtime::RuntimeCommandType;
using triwhirl::runtime::normalizeRuntimeCommandLine;
using triwhirl::runtime::parseRuntimeCommand;

void expectType(const char* text, const RuntimeCommandType type) {
  const auto parsed = parseRuntimeCommand(text);
  assert(parsed.status == RuntimeCommandParseStatus::kCommand);
  assert(parsed.command.type == type);
}

}  // namespace

int main() {
  {
    char normalized[64]{};
    const std::size_t length = normalizeRuntimeCommandLine(
        " \t motor   status \t ", normalized, sizeof(normalized));
    assert(length == std::strlen("motor status"));
    assert(std::strcmp(normalized, "motor status") == 0);
  }

  {
    char normalized[8]{};
    const std::size_t length = normalizeRuntimeCommandLine(
        "   motor stop   ", normalized, sizeof(normalized));
    assert(length == 7U);
    assert(std::strcmp(normalized, "motor s") == 0);
  }

  expectType("status", RuntimeCommandType::kStatus);
  expectType("motor status", RuntimeCommandType::kMotorStatus);
  expectType("imu status", RuntimeCommandType::kImuStatus);
  expectType("log status", RuntimeCommandType::kLogStatus);
  expectType("swing status", RuntimeCommandType::kSwingStatus);
  expectType("balance status", RuntimeCommandType::kBalanceStatus);
  expectType("balance start", RuntimeCommandType::kBalanceStart);
  expectType("balance stop", RuntimeCommandType::kBalanceStop);
  expectType("timing profile status", RuntimeCommandType::kTimingProfileStatus);
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
  expectType("log start", RuntimeCommandType::kLogStart);
  expectType("log critical on", RuntimeCommandType::kLogCriticalOn);
  expectType("log critical off", RuntimeCommandType::kLogCriticalOff);
  expectType("log stop", RuntimeCommandType::kLogStop);
  expectType("log dump", RuntimeCommandType::kLogDump);

  // Legacy strtok-based CLI accepted surrounding whitespace and arbitrary
  // horizontal whitespace between tokens. Keep that wire behavior while
  // parsing is supervisor-owned.
  expectType("   motor\t\tstop   ", RuntimeCommandType::kMotorStop);
  expectType("\t timing   profile\tstatus \t", RuntimeCommandType::kTimingProfileStatus);
  expectType("  imu   status  ", RuntimeCommandType::kImuStatus);
  expectType("  balance\t start ", RuntimeCommandType::kBalanceStart);

  {
    const auto parsed = parseRuntimeCommand("motor vq -0.625");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kMotorVq);
    assert(std::fabs(parsed.command.payload.motor_vq.volts + 0.625F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("  motor\t vq   -0.625  ");
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
    assert(std::fabs(parsed.command.payload.motor_calibrate.amplitude_v - 3.0F) < 1.0e-6F);
    assert(std::fabs(parsed.command.payload.motor_calibrate.electrical_hz - 1.0F) < 1.0e-6F);
    assert(std::fabs(parsed.command.payload.motor_calibrate.turns - 1.0F) < 1.0e-6F);
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
        "balance config 12.5 1.2 -0.08 68 6 24 1.2 40");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kBalanceConfig);
    const auto& config = parsed.command.payload.balance_config;
    assert(std::fabs(config.k_theta - 12.5F) < 1.0e-6F);
    assert(std::fabs(config.k_rate - 1.2F) < 1.0e-6F);
    assert(std::fabs(config.k_wheel + 0.08F) < 1.0e-6F);
    assert(std::fabs(config.theta_reference_deg - 68.0F) < 1.0e-6F);
    assert(std::fabs(config.capture_deg - 6.0F) < 1.0e-6F);
    assert(std::fabs(config.fall_deg - 24.0F) < 1.0e-6F);
    assert(std::fabs(config.vq_limit_v - 1.2F) < 1.0e-6F);
    assert(std::fabs(config.wheel_rate_limit_rad_s - 40.0F) < 1.0e-6F);
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
    const auto parsed = parseRuntimeCommand(
        "  swing\tconfig  24\t0.4 0.8  5 9 14\t12.5 0.2 -1 68 20   ");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kSwingConfig);
    assert(parsed.command.payload.swing_config.target_captures == 24U);
    assert(parsed.command.payload.swing_config.probe_duration_us == 12500U);
    assert(parsed.command.payload.swing_config.max_duration_us == 20000000U);
  }

  {
    const auto parsed = parseRuntimeCommand("log prepare");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kLogPrepare);
    assert(std::fabs(parsed.command.payload.log_prepare.seconds - 45.0F) < 1.0e-6F);
  }

  {
    const auto parsed = parseRuntimeCommand("log prepare 12.5");
    assert(parsed.status == RuntimeCommandParseStatus::kCommand);
    assert(parsed.command.type == RuntimeCommandType::kLogPrepare);
    assert(std::fabs(parsed.command.payload.log_prepare.seconds - 12.5F) < 1.0e-6F);
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
  }

  {
    const auto parsed = parseRuntimeCommand("imu map 0 1 2 1 -1");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
  }

  {
    const auto parsed = parseRuntimeCommand("field 1.0");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
  }

  {
    const auto parsed = parseRuntimeCommand("balance config 1 2 3 68 6 24 1.2");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
  }

  {
    const auto parsed = parseRuntimeCommand("swing config 24 0.4 0.8");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
  }

  {
    const auto parsed = parseRuntimeCommand("log critical maybe");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
  }

  {
    const auto parsed = parseRuntimeCommand("definitely unknown");
    assert(parsed.status == RuntimeCommandParseStatus::kNotMatched);
  }

  return 0;
}
