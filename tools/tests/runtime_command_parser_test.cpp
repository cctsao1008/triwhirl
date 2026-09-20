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
  expectType("timing reset", RuntimeCommandType::kTimingReset);
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
    const auto parsed = parseRuntimeCommand("motor vq");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
    assert(parsed.error != nullptr);
    assert(std::strcmp(parsed.error, "ERR usage: motor vq <volts>\r\n") == 0);
  }

  {
    const auto parsed = parseRuntimeCommand("field 1.0");
    assert(parsed.status == RuntimeCommandParseStatus::kUsageError);
    assert(parsed.error != nullptr);
  }

  {
    const auto parsed = parseRuntimeCommand("motor status");
    assert(parsed.status == RuntimeCommandParseStatus::kNotMatched);
  }

  return 0;
}
