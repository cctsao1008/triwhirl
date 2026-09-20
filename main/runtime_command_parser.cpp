#include "runtime_command_parser.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace triwhirl::runtime {
namespace {

constexpr std::size_t kCommandTextBytes = 128U;
constexpr std::uint32_t kDefaultGyroCalibrationSamples = 500U;
constexpr float kDefaultCalibrationAmplitudeV = 0.6F;
constexpr float kDefaultCalibrationElectricalHz = 0.5F;
constexpr float kDefaultCalibrationTurns = 4.0F;

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
  if (next != '\0' && next != ' ' && next != '\t') {
    return false;
  }
  const char* cursor = line + length;
  while (*cursor == ' ' || *cursor == '\t') {
    ++cursor;
  }
  *arguments = cursor;
  return true;
}

char* nextToken(char** const cursor) {
  if (cursor == nullptr || *cursor == nullptr) {
    return nullptr;
  }
  while (**cursor == ' ' || **cursor == '\t') {
    ++(*cursor);
  }
  if (**cursor == '\0') {
    return nullptr;
  }
  char* token = *cursor;
  while (**cursor != '\0' && **cursor != ' ' && **cursor != '\t') {
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

RuntimeCommandParseResult parseRuntimeCommand(const char* const line) {
  if (line == nullptr) {
    return {};
  }

  if (std::strcmp(line, "motor stop") == 0) {
    return commandResult(RuntimeCommandType::kMotorStop);
  }
  if (std::strcmp(line, "stop") == 0) {
    return commandResult(RuntimeCommandType::kStop);
  }
  if (std::strcmp(line, "swing abort") == 0) {
    return commandResult(RuntimeCommandType::kSwingAbort);
  }
  if (std::strcmp(line, "timing reset") == 0) {
    return commandResult(RuntimeCommandType::kTimingReset);
  }
  if (std::strcmp(line, "fault clear") == 0) {
    return commandResult(RuntimeCommandType::kFaultClear);
  }
  if (std::strcmp(line, "telemetry on") == 0) {
    return commandResult(RuntimeCommandType::kTelemetryOn);
  }
  if (std::strcmp(line, "telemetry off") == 0) {
    return commandResult(RuntimeCommandType::kTelemetryOff);
  }

  const char* arguments = nullptr;
  if (commandArguments(line, "motor vq", &arguments)) {
    if (*arguments == '\0') {
      return usageError("ERR usage: motor vq <volts>\r\n");
    }
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
    if (pole_pairs_token == nullptr || direction_token == nullptr ||
        offset_token == nullptr) {
      return usageError(
          "ERR usage: motor config <pole_pairs> <sensor_dir> <offset_rad>\r\n");
    }
    auto result = commandResult(RuntimeCommandType::kMotorConfig);
    result.command.payload.motor_config.pole_pairs = std::atoi(pole_pairs_token);
    result.command.payload.motor_config.sensor_direction = std::atoi(direction_token);
    result.command.payload.motor_config.electrical_offset_rad =
        std::strtof(offset_token, nullptr);
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
        amplitude_token == nullptr ? kDefaultCalibrationAmplitudeV
                                   : std::strtof(amplitude_token, nullptr);
    result.command.payload.motor_calibrate.electrical_hz =
        hz_token == nullptr ? kDefaultCalibrationElectricalHz
                            : std::strtof(hz_token, nullptr);
    result.command.payload.motor_calibrate.turns =
        turns_token == nullptr ? kDefaultCalibrationTurns
                               : std::strtof(turns_token, nullptr);
    return result;
  }

  if (commandArguments(line, "field", &arguments)) {
    char copy[kCommandTextBytes]{};
    std::snprintf(copy, sizeof(copy), "%s", arguments);
    char* cursor = copy;
    char* hz_token = nextToken(&cursor);
    char* amplitude_token = nextToken(&cursor);
    if (hz_token == nullptr || amplitude_token == nullptr) {
      return usageError("ERR usage: field <electrical_hz> <amplitude_v>\r\n");
    }
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
    result.command.payload.imu_calibrate.samples =
        *arguments == '\0'
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
    if (sin_axis_token == nullptr || cos_axis_token == nullptr ||
        gyro_axis_token == nullptr || sin_sign_token == nullptr ||
        cos_sign_token == nullptr || gyro_sign_token == nullptr) {
      return usageError(
          "ERR usage: imu map <sin_axis> <cos_axis> <gyro_axis> <sin_sign> <cos_sign> <gyro_sign>\r\n");
    }
    auto result = commandResult(RuntimeCommandType::kImuMap);
    result.command.payload.imu_map.accel_sin_axis = std::atoi(sin_axis_token);
    result.command.payload.imu_map.accel_cos_axis = std::atoi(cos_axis_token);
    result.command.payload.imu_map.gyro_axis = std::atoi(gyro_axis_token);
    result.command.payload.imu_map.accel_sin_sign = std::atoi(sin_sign_token);
    result.command.payload.imu_map.accel_cos_sign = std::atoi(cos_sign_token);
    result.command.payload.imu_map.gyro_sign = std::atoi(gyro_sign_token);
    return result;
  }

  return {};
}

}  // namespace triwhirl::runtime
