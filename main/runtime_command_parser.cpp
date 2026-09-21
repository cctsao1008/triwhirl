#include "runtime_command_parser.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace triwhirl::runtime {
namespace {

constexpr std::size_t kCommandTextBytes = 128U;
constexpr std::uint32_t kDefaultGyroCalibrationSamples = 500U;
constexpr float kDefaultCalibrationAmplitudeV = 0.6F;
constexpr float kDefaultCalibrationElectricalHz = 0.5F;
constexpr float kDefaultCalibrationTurns = 4.0F;
constexpr float kDefaultLogSeconds = 45.0F;
constexpr const char* kSwingConfigUsage =
    "ERR usage: swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>\r\n";

bool commandArguments(const char* const line, const char* const command,
                      const char** const arguments) {
  if (line == nullptr || command == nullptr || arguments == nullptr) {
    return false;
  }
  const std::size_t length = std::strlen(command);
  if (std::strncmp(line, command, length) != 0) {
    return false;
  }
  const char next = line[length];
  if (next != '\0' && next != ' ') {
    return false;
  }
  const char* cursor = line + length;
  while (*cursor == ' ') {
    ++cursor;
  }
  *arguments = cursor;
  return true;
}

char* nextToken(char** const cursor) {
  if (cursor == nullptr || *cursor == nullptr) {
    return nullptr;
  }
  while (**cursor == ' ') {
    ++(*cursor);
  }
  if (**cursor == '\0') {
    return nullptr;
  }
  char* token = *cursor;
  while (**cursor != '\0' && **cursor != ' ') {
    ++(*cursor);
  }
  if (**cursor != '\0') {
    **cursor = '\0';
    ++(*cursor);
  }
  return token;
}

RuntimeCommandParseResult commandResult(const RuntimeCommandType type) {
  RuntimeCommandParseResult result{};
  result.status = RuntimeCommandParseStatus::kCommand;
  result.command.type = type;
  return result;
}

RuntimeCommandParseResult usageError(const char* const error) {
  RuntimeCommandParseResult result{};
  result.status = RuntimeCommandParseStatus::kUsageError;
  result.error = error;
  return result;
}

}  // namespace

std::size_t normalizeRuntimeCommandLine(const char* input, char* output,
                                        const std::size_t output_bytes) {
  if (output == nullptr || output_bytes == 0U) {
    return 0U;
  }
  output[0] = '\0';
  if (input == nullptr) {
    return 0U;
  }

  std::size_t written = 0U;
  bool pending_space = false;
  while (*input != '\0' && written + 1U < output_bytes) {
    if (*input == ' ' || *input == '\t') {
      pending_space = written > 0U;
      ++input;
      continue;
    }
    if (pending_space && written + 1U < output_bytes) {
      output[written++] = ' ';
    }
    pending_space = false;
    output[written++] = *input++;
  }
  output[written] = '\0';
  return written;
}

RuntimeCommandParseResult parseRuntimeCommand(const char* const input) {
  if (input == nullptr) {
    return {};
  }

  char normalized[kCommandTextBytes]{};
  normalizeRuntimeCommandLine(input, normalized, sizeof(normalized));
  const char* const line = normalized;

  if (std::strcmp(line, "status") == 0) return commandResult(RuntimeCommandType::kStatus);
  if (std::strcmp(line, "motor status") == 0) return commandResult(RuntimeCommandType::kMotorStatus);
  if (std::strcmp(line, "imu") == 0 || std::strcmp(line, "imu status") == 0)
    return commandResult(RuntimeCommandType::kImuStatus);
  if (std::strcmp(line, "log") == 0 || std::strcmp(line, "log status") == 0)
    return commandResult(RuntimeCommandType::kLogStatus);
  if (std::strcmp(line, "swing") == 0 || std::strcmp(line, "swing status") == 0)
    return commandResult(RuntimeCommandType::kSwingStatus);
  if (std::strcmp(line, "timing profile") == 0 ||
      std::strcmp(line, "timing profile status") == 0)
    return commandResult(RuntimeCommandType::kTimingProfileStatus);
  if (std::strcmp(line, "motor stop") == 0) return commandResult(RuntimeCommandType::kMotorStop);
  if (std::strcmp(line, "stop") == 0) return commandResult(RuntimeCommandType::kStop);
  if (std::strcmp(line, "swing abort") == 0) return commandResult(RuntimeCommandType::kSwingAbort);
  if (std::strcmp(line, "swing start") == 0) return commandResult(RuntimeCommandType::kSwingStart);
  if (std::strcmp(line, "timing reset") == 0) return commandResult(RuntimeCommandType::kTimingReset);
  if (std::strcmp(line, "timing profile on") == 0) return commandResult(RuntimeCommandType::kTimingProfileOn);
  if (std::strcmp(line, "timing profile off") == 0) return commandResult(RuntimeCommandType::kTimingProfileOff);
  if (std::strcmp(line, "timing profile reset") == 0) return commandResult(RuntimeCommandType::kTimingProfileReset);
  if (std::strcmp(line, "fault clear") == 0) return commandResult(RuntimeCommandType::kFaultClear);
  if (std::strcmp(line, "telemetry on") == 0) return commandResult(RuntimeCommandType::kTelemetryOn);
  if (std::strcmp(line, "telemetry off") == 0) return commandResult(RuntimeCommandType::kTelemetryOff);
  if (std::strcmp(line, "log start") == 0) return commandResult(RuntimeCommandType::kLogStart);
  if (std::strcmp(line, "log critical on") == 0) return commandResult(RuntimeCommandType::kLogCriticalOn);
  if (std::strcmp(line, "log critical off") == 0) return commandResult(RuntimeCommandType::kLogCriticalOff);
  if (std::strcmp(line, "log stop") == 0) return commandResult(RuntimeCommandType::kLogStop);
  if (std::strcmp(line, "log dump") == 0) return commandResult(RuntimeCommandType::kLogDump);

  const char* arguments = nullptr;
  if (commandArguments(line, "motor vq", &arguments)) {
    if (*arguments == '\0') return usageError("ERR usage: motor vq <volts>\r\n");
    auto result = commandResult(RuntimeCommandType::kMotorVq);
    result.command.payload.motor_vq.volts = std::strtof(arguments, nullptr);
    return result;
  }

  if (commandArguments(line, "motor config", &arguments)) {
    char copy[kCommandTextBytes]{};
    std::snprintf(copy, sizeof(copy), "%s", arguments);
    char* cursor = copy;
    char* pole_pairs_token = nextToken(&cursor);
    char* direction_token = nextToken(&cursor);
    char* offset_token = nextToken(&cursor);
    if (pole_pairs_token == nullptr || direction_token == nullptr || offset_token == nullptr)
      return usageError("ERR usage: motor config <pole_pairs> <sensor_dir> <offset_rad>\r\n");
    auto result = commandResult(RuntimeCommandType::kMotorConfig);
    result.command.payload.motor_config.pole_pairs = std::atoi(pole_pairs_token);
    result.command.payload.motor_config.sensor_direction = std::atoi(direction_token);
    result.command.payload.motor_config.electrical_offset_rad = std::strtof(offset_token, nullptr);
    return result;
  }

  if (commandArguments(line, "motor calibrate", &arguments)) {
    char copy[kCommandTextBytes]{};
    std::snprintf(copy, sizeof(copy), "%s", arguments);
    char* cursor = copy;
    char* amplitude_token = nextToken(&cursor);
    char* hz_token = nextToken(&cursor);
    char* turns_token = nextToken(&cursor);
    auto result = commandResult(RuntimeCommandType::kMotorCalibrate);
    result.command.payload.motor_calibrate.amplitude_v =
        amplitude_token == nullptr ? kDefaultCalibrationAmplitudeV : std::strtof(amplitude_token, nullptr);
    result.command.payload.motor_calibrate.electrical_hz =
        hz_token == nullptr ? kDefaultCalibrationElectricalHz : std::strtof(hz_token, nullptr);
    result.command.payload.motor_calibrate.turns =
        turns_token == nullptr ? kDefaultCalibrationTurns : std::strtof(turns_token, nullptr);
    return result;
  }

  if (commandArguments(line, "field", &arguments)) {
    char copy[kCommandTextBytes]{};
    std::snprintf(copy, sizeof(copy), "%s", arguments);
    char* cursor = copy;
    char* hz_token = nextToken(&cursor);
    char* amplitude_token = nextToken(&cursor);
    if (hz_token == nullptr || amplitude_token == nullptr)
      return usageError("ERR usage: field <electrical_hz> <amplitude_v>\r\n");
    auto result = commandResult(RuntimeCommandType::kField);
    result.command.payload.field.electrical_hz = std::strtof(hz_token, nullptr);
    result.command.payload.field.amplitude_v = std::strtof(amplitude_token, nullptr);
    return result;
  }

  if (commandArguments(line, "attitude reset", &arguments)) {
    auto result = commandResult(RuntimeCommandType::kAttitudeReset);
    if (*arguments == '\0') {
      result.command.payload.attitude_reset.use_accelerometer = true;
    } else {
      result.command.payload.attitude_reset.use_accelerometer = false;
      result.command.payload.attitude_reset.angle_rad = std::strtof(arguments, nullptr);
    }
    return result;
  }

  if (commandArguments(line, "imu calibrate", &arguments)) {
    auto result = commandResult(RuntimeCommandType::kImuCalibrate);
    result.command.payload.imu_calibrate.samples = *arguments == '\0'
        ? kDefaultGyroCalibrationSamples
        : static_cast<std::uint32_t>(std::strtoul(arguments, nullptr, 10));
    return result;
  }

  if (commandArguments(line, "imu map", &arguments)) {
    char copy[kCommandTextBytes]{};
    std::snprintf(copy, sizeof(copy), "%s", arguments);
    char* cursor = copy;
    char* sin_axis_token = nextToken(&cursor);
    char* cos_axis_token = nextToken(&cursor);
    char* gyro_axis_token = nextToken(&cursor);
    char* sin_sign_token = nextToken(&cursor);
    char* cos_sign_token = nextToken(&cursor);
    char* gyro_sign_token = nextToken(&cursor);
    if (sin_axis_token == nullptr || cos_axis_token == nullptr || gyro_axis_token == nullptr ||
        sin_sign_token == nullptr || cos_sign_token == nullptr || gyro_sign_token == nullptr)
      return usageError("ERR usage: imu map <sin_axis> <cos_axis> <gyro_axis> <sin_sign> <cos_sign> <gyro_sign>\r\n");
    auto result = commandResult(RuntimeCommandType::kImuMap);
    result.command.payload.imu_map.accel_sin_axis = std::atoi(sin_axis_token);
    result.command.payload.imu_map.accel_cos_axis = std::atoi(cos_axis_token);
    result.command.payload.imu_map.gyro_axis = std::atoi(gyro_axis_token);
    result.command.payload.imu_map.accel_sin_sign = std::atoi(sin_sign_token);
    result.command.payload.imu_map.accel_cos_sign = std::atoi(cos_sign_token);
    result.command.payload.imu_map.gyro_sign = std::atoi(gyro_sign_token);
    return result;
  }

  if (commandArguments(line, "swing config", &arguments)) {
    char copy[kCommandTextBytes]{};
    std::snprintf(copy, sizeof(copy), "%s", arguments);
    char* cursor = copy;
    char* target = nextToken(&cursor);
    char* pump_low = nextToken(&cursor);
    char* pump_high = nextToken(&cursor);
    char* capture = nextToken(&cursor);
    char* probe_exit = nextToken(&cursor);
    char* rearm = nextToken(&cursor);
    char* probe_ms = nextToken(&cursor);
    char* rate_switch = nextToken(&cursor);
    char* polarity = nextToken(&cursor);
    char* vertex_a = nextToken(&cursor);
    char* max_s = nextToken(&cursor);
    if (target == nullptr || pump_low == nullptr || pump_high == nullptr || capture == nullptr ||
        probe_exit == nullptr || rearm == nullptr || probe_ms == nullptr || rate_switch == nullptr ||
        polarity == nullptr || vertex_a == nullptr || max_s == nullptr || nextToken(&cursor) != nullptr)
      return usageError(kSwingConfigUsage);

    const double probe_ms_value = std::strtod(probe_ms, nullptr);
    const double max_s_value = std::strtod(max_s, nullptr);
    if (!std::isfinite(probe_ms_value) || probe_ms_value <= 0.0 ||
        probe_ms_value > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) * 1.0e-3 ||
        !std::isfinite(max_s_value) || max_s_value <= 0.0 ||
        max_s_value > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) * 1.0e-6)
      return usageError(kSwingConfigUsage);

    auto result = commandResult(RuntimeCommandType::kSwingConfig);
    auto& config = result.command.payload.swing_config;
    config.target_captures = static_cast<std::uint32_t>(std::strtoul(target, nullptr, 10));
    config.pump_v_low = std::strtof(pump_low, nullptr);
    config.pump_v_high = std::strtof(pump_high, nullptr);
    config.capture_deg = std::strtof(capture, nullptr);
    config.probe_exit_deg = std::strtof(probe_exit, nullptr);
    config.rearm_deg = std::strtof(rearm, nullptr);
    config.probe_duration_us = static_cast<std::uint32_t>(std::llround(probe_ms_value * 1000.0));
    config.rate_switch_rad_s = std::strtof(rate_switch, nullptr);
    config.pump_polarity = std::atoi(polarity);
    config.vertex_a_deg = std::strtof(vertex_a, nullptr);
    config.max_duration_us = static_cast<std::uint32_t>(std::llround(max_s_value * 1000000.0));
    return result;
  }

  if (commandArguments(line, "log prepare", &arguments)) {
    auto result = commandResult(RuntimeCommandType::kLogPrepare);
    result.command.payload.log_prepare.seconds =
        *arguments == '\0' ? kDefaultLogSeconds : std::strtof(arguments, nullptr);
    return result;
  }

  if (commandArguments(line, "log critical", &arguments))
    return usageError("ERR usage: log critical <on|off>\r\n");
  if (commandArguments(line, "log", &arguments))
    return usageError("ERR usage: log <status|prepare [seconds]|start|critical on|off|stop|dump>\r\n");
  if (commandArguments(line, "motor", &arguments))
    return usageError("ERR usage: motor <calibrate|config|vq|status|stop>\r\n");
  if (commandArguments(line, "imu", &arguments))
    return usageError("ERR usage: imu <status|calibrate [samples]|map ...>\r\n");
  if (commandArguments(line, "attitude", &arguments))
    return usageError("ERR usage: attitude <status|reset [angle_rad]>\r\n");
  if (commandArguments(line, "timing profile", &arguments))
    return usageError("ERR usage: timing profile <status|on|off|reset>\r\n");
  if (commandArguments(line, "timing", &arguments))
    return usageError("ERR usage: timing <status|reset>\r\n");
  if (commandArguments(line, "fault", &arguments))
    return usageError("ERR usage: fault <status|clear>\r\n");
  if (commandArguments(line, "ble", &arguments))
    return usageError("ERR usage: ble status\r\n");
  if (commandArguments(line, "swing", &arguments))
    return usageError("ERR usage: swing <status|config ...|start|abort>\r\n");
  if (commandArguments(line, "telemetry", &arguments))
    return usageError("ERR usage: telemetry [on|off]\r\n");

  return {};
}

}  // namespace triwhirl::runtime
