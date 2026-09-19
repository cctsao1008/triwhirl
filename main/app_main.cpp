#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "triwhirl/board.hpp"
#include "triwhirl/drivers/as5600.hpp"
#include "triwhirl/motor/three_pwm_bridge.hpp"
#include "triwhirl/voltage_mode_foc.hpp"
#include "triwhirl/wheel_kinematics.hpp"

namespace {

using triwhirl::MotorElectricalConfig;
using triwhirl::PhaseVoltages;
using triwhirl::WheelKinematics;
using triwhirl::WheelKinematicsState;
using triwhirl::drivers::As5600;
using triwhirl::drivers::As5600Status;
using triwhirl::motor::ThreePwmBridge;

constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kMaxElectricalHz = 30.0F;
constexpr float kWheelVelocityFilterTauS = 0.01F;
constexpr float kMotorVectorLimitV = triwhirl::board::kBringupPhaseAmplitudeMaxV;
constexpr float kDefaultCalibrationAmplitudeV = 0.6F;
constexpr float kDefaultCalibrationElectricalHz = 0.5F;
constexpr float kDefaultCalibrationTurns = 4.0F;
constexpr std::uint32_t kCalibrationAlignUs = 500000U;
constexpr std::uint32_t kCalibrationSettleUs = 400000U;
constexpr std::uint32_t kEncoderSamplePeriodUs = 1000U;
constexpr std::uint32_t kEncoderHealthPeriodUs = 100000U;
constexpr std::uint32_t kTelemetryPeriodUs = 20000U;
constexpr std::uint32_t kPwmFrequencyHz = 25000U;

enum class MotorMode {
  kStopped,
  kOpenLoop,
  kFoc,
  kCalibrating,
};

enum class CalibrationStage {
  kIdle,
  kAlign,
  kSweep,
  kSettle,
};

struct CalibrationState {
  CalibrationStage stage = CalibrationStage::kIdle;
  float amplitude_v = kDefaultCalibrationAmplitudeV;
  float electrical_hz = kDefaultCalibrationElectricalHz;
  float electrical_turns = kDefaultCalibrationTurns;
  float start_mechanical_rad = 0.0F;
  float commanded_electrical_rad = 0.0F;
  std::uint32_t stage_start_us = 0U;
};

As5600 encoder;
WheelKinematics wheel_kinematics(kWheelVelocityFilterTauS);
ThreePwmBridge bridge;

As5600Status encoder_status{};
WheelKinematicsState wheel_state{};
bool encoder_status_valid = false;
bool encoder_sample_valid = false;
std::uint32_t encoder_read_errors = 0U;

MotorMode motor_mode = MotorMode::kStopped;
MotorElectricalConfig motor_config{};
bool motor_config_valid = false;
CalibrationState calibration{};

bool telemetry_enabled = false;
float open_loop_amplitude_v = 0.0F;
float open_loop_hz = 0.0F;
float open_loop_angle_rad = 0.0F;
float vq_command_v = 0.0F;
float electrical_angle_rad = 0.0F;
std::uint32_t last_motor_update_us = 0U;
std::uint32_t last_encoder_sample_us = 0U;
std::uint32_t last_encoder_health_us = 0U;
std::uint32_t last_telemetry_us = 0U;

char command_line[128]{};
std::size_t command_length = 0U;

void consoleWrite(const char* text) {
  if (text == nullptr) {
    return;
  }
  uart_write_bytes(UART_NUM_0, text, std::strlen(text));
}

void consolePrintf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  const int length = std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (length <= 0) {
    return;
  }
  const std::size_t count = static_cast<std::size_t>(length) < sizeof(buffer)
                                ? static_cast<std::size_t>(length)
                                : sizeof(buffer) - 1U;
  uart_write_bytes(UART_NUM_0, buffer, count);
}

const char* motorModeName(const MotorMode mode) {
  switch (mode) {
    case MotorMode::kStopped:
      return "stopped";
    case MotorMode::kOpenLoop:
      return "open_loop";
    case MotorMode::kFoc:
      return "foc";
    case MotorMode::kCalibrating:
      return "calibrating";
  }
  return "unknown";
}

void printPrompt() {
  if (!telemetry_enabled) {
    consoleWrite("> ");
  }
}

float clampFinite(const float value, const float low, const float high) {
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  return value < low ? low : (value > high ? high : value);
}

bool refreshEncoderHealth() {
  As5600Status status{};
  if (!encoder.readStatus(&status)) {
    encoder_status_valid = false;
    return false;
  }
  encoder_status = status;
  encoder_status_valid = true;
  return true;
}

bool sampleEncoder(const std::uint32_t sample_time_us) {
  std::uint16_t raw_count = 0U;
  if (!encoder.readRawAngle(&raw_count)) {
    encoder_sample_valid = false;
    ++encoder_read_errors;
    return false;
  }
  wheel_state = wheel_kinematics.update(raw_count, sample_time_us);
  encoder_sample_valid = true;
  return true;
}

void stopMotor() {
  motor_mode = MotorMode::kStopped;
  calibration.stage = CalibrationStage::kIdle;
  open_loop_amplitude_v = 0.0F;
  open_loop_hz = 0.0F;
  vq_command_v = 0.0F;
  bridge.stopZeroVector();
}

void applyDq(const float electrical_angle, const float vd_v, const float vq_v) {
  const PhaseVoltages phase = triwhirl::makeDqVoltage(
      electrical_angle, vd_v, vq_v, triwhirl::board::kMotorBusNominalV,
      kMotorVectorLimitV);
  bridge.setPhaseVoltages(phase.a, phase.b, phase.c);
}

void updateEncoder(const std::uint32_t now_us) {
  if ((now_us - last_encoder_sample_us) >= kEncoderSamplePeriodUs) {
    last_encoder_sample_us = now_us;
    sampleEncoder(now_us);
  }
  if ((now_us - last_encoder_health_us) >= kEncoderHealthPeriodUs) {
    last_encoder_health_us = now_us;
    refreshEncoderHealth();
  }
}

void finishCalibration() {
  const float delta_mechanical =
      wheel_state.unwrapped_angle_rad - calibration.start_mechanical_rad;
  const float total_electrical = calibration.electrical_turns * kTwoPi;
  const float mechanical_travel = std::fabs(delta_mechanical);

  if (!(mechanical_travel > 0.05F) || !std::isfinite(mechanical_travel)) {
    stopMotor();
    consoleWrite("ERR motor calibration: no usable mechanical motion\r\n");
    return;
  }

  const float pole_pairs_estimate = total_electrical / mechanical_travel;
  const int pole_pairs = static_cast<int>(std::lround(pole_pairs_estimate));
  if (pole_pairs < 1 || pole_pairs > 64 ||
      std::fabs(pole_pairs_estimate - static_cast<float>(pole_pairs)) > 0.45F) {
    stopMotor();
    consolePrintf("ERR motor calibration: pole-pair estimate %.3f is invalid\r\n",
                  pole_pairs_estimate);
    return;
  }

  const int sensor_direction = delta_mechanical >= 0.0F ? 1 : -1;
  const float final_electrical =
      triwhirl::wrapElectricalAngle(calibration.commanded_electrical_rad);
  const float offset = triwhirl::wrapElectricalAngle(
      final_electrical -
      static_cast<float>(sensor_direction * pole_pairs) *
          wheel_state.unwrapped_angle_rad);

  motor_config.pole_pairs = pole_pairs;
  motor_config.sensor_direction = sensor_direction;
  motor_config.electrical_offset_rad = offset;
  motor_config_valid = triwhirl::validMotorElectricalConfig(motor_config);

  stopMotor();
  consolePrintf(
      "OK motor calibrated pole_pairs=%d sensor_dir=%d offset_rad=%.6f estimate=%.3f\r\n",
      motor_config.pole_pairs, motor_config.sensor_direction,
      motor_config.electrical_offset_rad, pole_pairs_estimate);
}

void updateCalibration(const std::uint32_t now_us) {
  if (calibration.stage == CalibrationStage::kIdle) {
    stopMotor();
    return;
  }

  if (!encoder_sample_valid) {
    stopMotor();
    consoleWrite("ERR motor calibration: encoder read unavailable\r\n");
    return;
  }

  if (calibration.stage == CalibrationStage::kAlign) {
    calibration.commanded_electrical_rad = 0.0F;
    applyDq(0.0F, calibration.amplitude_v, 0.0F);
    if ((now_us - calibration.stage_start_us) >= kCalibrationAlignUs) {
      calibration.start_mechanical_rad = wheel_state.unwrapped_angle_rad;
      calibration.stage = CalibrationStage::kSweep;
      calibration.stage_start_us = now_us;
    }
    return;
  }

  const float total_electrical = calibration.electrical_turns * kTwoPi;
  if (calibration.stage == CalibrationStage::kSweep) {
    const float elapsed_s =
        static_cast<float>(now_us - calibration.stage_start_us) * 1.0e-6F;
    calibration.commanded_electrical_rad =
        kTwoPi * calibration.electrical_hz * elapsed_s;
    if (calibration.commanded_electrical_rad >= total_electrical) {
      calibration.commanded_electrical_rad = total_electrical;
      calibration.stage = CalibrationStage::kSettle;
      calibration.stage_start_us = now_us;
    }
    applyDq(calibration.commanded_electrical_rad,
            calibration.amplitude_v, 0.0F);
    return;
  }

  if (calibration.stage == CalibrationStage::kSettle) {
    applyDq(calibration.commanded_electrical_rad,
            calibration.amplitude_v, 0.0F);
    if ((now_us - calibration.stage_start_us) >= kCalibrationSettleUs) {
      finishCalibration();
    }
  }
}

void updateMotor(const std::uint32_t now_us) {
  const std::uint32_t elapsed_us = now_us - last_motor_update_us;
  last_motor_update_us = now_us;

  if (motor_mode == MotorMode::kStopped) {
    return;
  }

  if (motor_mode == MotorMode::kCalibrating) {
    updateCalibration(now_us);
    return;
  }

  if (motor_mode == MotorMode::kOpenLoop) {
    const float dt = static_cast<float>(elapsed_us) * 1.0e-6F;
    open_loop_angle_rad = triwhirl::wrapElectricalAngle(
        open_loop_angle_rad + kTwoPi * open_loop_hz * dt);
    electrical_angle_rad = open_loop_angle_rad;
    applyDq(open_loop_angle_rad, open_loop_amplitude_v, 0.0F);
    return;
  }

  if (motor_mode == MotorMode::kFoc) {
    if (!motor_config_valid || !encoder_sample_valid) {
      stopMotor();
      consoleWrite("ERR FOC stopped: motor configuration or encoder unavailable\r\n");
      return;
    }
    electrical_angle_rad = triwhirl::electricalAngleFromMechanical(
        wheel_state.unwrapped_angle_rad, motor_config);
    applyDq(electrical_angle_rad, 0.0F, vq_command_v);
  }
}

void printHelp() {
  consoleWrite("commands:\r\n");
  consoleWrite("  motor calibrate [amplitude_v] [electrical_hz] [turns]\r\n");
  consoleWrite("  motor config <pole_pairs> <sensor_dir> <offset_rad>\r\n");
  consoleWrite("  motor vq <volts>\r\n");
  consoleWrite("  motor status\r\n");
  consoleWrite("  motor stop\r\n");
  consoleWrite("  field <electrical_hz> <amplitude_v>\r\n");
  consoleWrite("  stop\r\n");
  consoleWrite("  status\r\n");
  consoleWrite("  telemetry [on|off]\r\n");
  consoleWrite("  help\r\n");
}

void printStatus() {
  refreshEncoderHealth();
  consolePrintf(
      "status,mode=%s,telemetry=%d,vq_v=%.6f,e_hz=%.6f,amp_v=%.6f,config=%d,pole_pairs=%d,sensor_dir=%d,offset_rad=%.6f,e_angle_rad=%.6f,status_ok=%d,sample_ok=%d,mag=%d,ml=%d,mh=%d,raw=%u,unwrapped_count=%lld,angle_rad=%.6f,unwrapped_rad=%.6f,vel_rad_s=%.6f,vel_inst_rad_s=%.6f,vel_valid=%d,read_errors=%lu\r\n",
      motorModeName(motor_mode), telemetry_enabled ? 1 : 0, vq_command_v,
      open_loop_hz, open_loop_amplitude_v, motor_config_valid ? 1 : 0,
      motor_config.pole_pairs, motor_config.sensor_direction,
      motor_config.electrical_offset_rad, electrical_angle_rad,
      encoder_status_valid ? 1 : 0, encoder_sample_valid ? 1 : 0,
      encoder_status.magnet_detected ? 1 : 0,
      encoder_status.magnet_too_weak ? 1 : 0,
      encoder_status.magnet_too_strong ? 1 : 0,
      static_cast<unsigned>(wheel_state.raw_count),
      static_cast<long long>(wheel_state.unwrapped_count), wheel_state.angle_rad,
      wheel_state.unwrapped_angle_rad, wheel_state.velocity_rad_s,
      wheel_state.instantaneous_velocity_rad_s,
      wheel_state.velocity_valid ? 1 : 0,
      static_cast<unsigned long>(encoder_read_errors));
}

void startCalibration(char* amplitude_token, char* hz_token, char* turns_token) {
  if (!encoder_sample_valid) {
    consoleWrite("ERR motor calibrate: encoder read unavailable\r\n");
    return;
  }

  calibration.amplitude_v = amplitude_token == nullptr
                                ? kDefaultCalibrationAmplitudeV
                                : clampFinite(std::strtof(amplitude_token, nullptr),
                                              0.1F, kMotorVectorLimitV);
  calibration.electrical_hz = hz_token == nullptr
                                  ? kDefaultCalibrationElectricalHz
                                  : clampFinite(std::strtof(hz_token, nullptr),
                                                0.1F, 2.0F);
  calibration.electrical_turns = turns_token == nullptr
                                     ? kDefaultCalibrationTurns
                                     : clampFinite(std::strtof(turns_token, nullptr),
                                                   1.0F, 12.0F);
  calibration.commanded_electrical_rad = 0.0F;
  calibration.start_mechanical_rad = wheel_state.unwrapped_angle_rad;
  calibration.stage = CalibrationStage::kAlign;
  calibration.stage_start_us = static_cast<std::uint32_t>(esp_timer_get_time());
  motor_mode = MotorMode::kCalibrating;
  vq_command_v = 0.0F;
  consolePrintf("OK motor calibration started amp_v=%.3f e_hz=%.3f turns=%.3f\r\n",
                calibration.amplitude_v, calibration.electrical_hz,
                calibration.electrical_turns);
}

void handleMotorCommand() {
  char* action = std::strtok(nullptr, " \t");
  if (action == nullptr) {
    consoleWrite("ERR usage: motor <calibrate|config|vq|status|stop>\r\n");
    return;
  }

  if (std::strcmp(action, "stop") == 0) {
    stopMotor();
    consoleWrite("OK motor stop\r\n");
    return;
  }

  if (std::strcmp(action, "status") == 0) {
    printStatus();
    return;
  }

  if (std::strcmp(action, "config") == 0) {
    char* pole_pairs_token = std::strtok(nullptr, " \t");
    char* direction_token = std::strtok(nullptr, " \t");
    char* offset_token = std::strtok(nullptr, " \t");
    if (pole_pairs_token == nullptr || direction_token == nullptr ||
        offset_token == nullptr) {
      consoleWrite("ERR usage: motor config <pole_pairs> <sensor_dir> <offset_rad>\r\n");
      return;
    }
    MotorElectricalConfig config{};
    config.pole_pairs = std::atoi(pole_pairs_token);
    config.sensor_direction = std::atoi(direction_token);
    config.electrical_offset_rad =
        triwhirl::wrapElectricalAngle(std::strtof(offset_token, nullptr));
    if (!triwhirl::validMotorElectricalConfig(config)) {
      consoleWrite("ERR invalid motor config\r\n");
      return;
    }
    stopMotor();
    motor_config = config;
    motor_config_valid = true;
    consolePrintf("OK motor config pole_pairs=%d sensor_dir=%d offset_rad=%.6f\r\n",
                  motor_config.pole_pairs, motor_config.sensor_direction,
                  motor_config.electrical_offset_rad);
    return;
  }

  if (std::strcmp(action, "vq") == 0) {
    char* vq_token = std::strtok(nullptr, " \t");
    if (vq_token == nullptr) {
      consoleWrite("ERR usage: motor vq <volts>\r\n");
      return;
    }
    if (!motor_config_valid) {
      consoleWrite("ERR motor is not calibrated/configured\r\n");
      return;
    }
    if (!encoder_sample_valid) {
      consoleWrite("ERR encoder read unavailable\r\n");
      return;
    }
    const float requested_vq =
        clampFinite(std::strtof(vq_token, nullptr), -kMotorVectorLimitV,
                    kMotorVectorLimitV);
    if (std::fabs(requested_vq) < 1.0e-4F) {
      stopMotor();
      consoleWrite("OK motor stop\r\n");
      return;
    }
    vq_command_v = requested_vq;
    motor_mode = MotorMode::kFoc;
    consolePrintf("OK motor FOC vq_v=%.6f\r\n", vq_command_v);
    return;
  }

  if (std::strcmp(action, "calibrate") == 0) {
    char* amplitude_token = std::strtok(nullptr, " \t");
    char* hz_token = std::strtok(nullptr, " \t");
    char* turns_token = std::strtok(nullptr, " \t");
    stopMotor();
    startCalibration(amplitude_token, hz_token, turns_token);
    return;
  }

  consoleWrite("ERR usage: motor <calibrate|config|vq|status|stop>\r\n");
}

void handleCommand(char* line) {
  char* command = std::strtok(line, " \t");
  if (command == nullptr) {
    return;
  }

  if (std::strcmp(command, "motor") == 0) {
    handleMotorCommand();
    return;
  }

  if (std::strcmp(command, "stop") == 0) {
    stopMotor();
    consoleWrite("OK stop\r\n");
    return;
  }

  if (std::strcmp(command, "status") == 0) {
    printStatus();
    return;
  }

  if (std::strcmp(command, "telemetry") == 0) {
    char* mode = std::strtok(nullptr, " \t");
    if (mode == nullptr) {
      consolePrintf("telemetry=%s\r\n", telemetry_enabled ? "on" : "off");
      return;
    }
    if (std::strcmp(mode, "on") == 0) {
      telemetry_enabled = true;
      last_telemetry_us = static_cast<std::uint32_t>(esp_timer_get_time());
      consoleWrite("OK telemetry on\r\n");
      return;
    }
    if (std::strcmp(mode, "off") == 0) {
      telemetry_enabled = false;
      consoleWrite("OK telemetry off\r\n");
      return;
    }
    consoleWrite("ERR usage: telemetry [on|off]\r\n");
    return;
  }

  if (std::strcmp(command, "help") == 0) {
    printHelp();
    return;
  }

  if (std::strcmp(command, "field") == 0) {
    char* hz_token = std::strtok(nullptr, " \t");
    char* amplitude_token = std::strtok(nullptr, " \t");
    if (hz_token == nullptr || amplitude_token == nullptr) {
      consoleWrite("ERR usage: field <electrical_hz> <amplitude_v>\r\n");
      return;
    }
    const float requested_hz =
        clampFinite(std::strtof(hz_token, nullptr), -kMaxElectricalHz,
                    kMaxElectricalHz);
    const float requested_amplitude =
        clampFinite(std::strtof(amplitude_token, nullptr), 0.0F,
                    kMotorVectorLimitV);
    if (requested_amplitude <= 0.0F || requested_hz == 0.0F) {
      stopMotor();
      consoleWrite("OK field stopped\r\n");
      return;
    }
    stopMotor();
    open_loop_hz = requested_hz;
    open_loop_amplitude_v = requested_amplitude;
    open_loop_angle_rad = 0.0F;
    motor_mode = MotorMode::kOpenLoop;
    consolePrintf("OK field e_hz=%.6f amp_v=%.6f\r\n", open_loop_hz,
                  open_loop_amplitude_v);
    return;
  }

  consoleWrite("ERR unknown command\r\n");
}

void pollConsole() {
  std::uint8_t input[32];
  const int received = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
  for (int i = 0; i < received; ++i) {
    const char c = static_cast<char>(input[i]);

    if (c == '\r' || c == '\n') {
      if (command_length > 0U) {
        consoleWrite("\r\n");
        command_line[command_length] = '\0';
        handleCommand(command_line);
        command_length = 0U;
        printPrompt();
      }
      continue;
    }

    if (c == '\b' || static_cast<unsigned char>(c) == 0x7FU) {
      if (command_length > 0U) {
        --command_length;
        consoleWrite("\b \b");
      }
      continue;
    }

    if (c < 0x20 || static_cast<unsigned char>(c) > 0x7EU) {
      continue;
    }

    if (command_length + 1U < sizeof(command_line)) {
      command_line[command_length++] = c;
      uart_write_bytes(UART_NUM_0, &c, 1U);
    } else {
      command_length = 0U;
      consoleWrite("\r\nERR command too long\r\n");
      printPrompt();
    }
  }
}

void emitTelemetry(const std::uint32_t now_us) {
  if (!telemetry_enabled ||
      (now_us - last_telemetry_us) < kTelemetryPeriodUs) {
    return;
  }
  last_telemetry_us = now_us;
  consolePrintf(
      "telemetry,%lu,%s,%.6f,%.6f,%.6f,%d,%d,%d,%u,%lld,%.6f,%.6f,%.6f,%.6f,%d,%lu\r\n",
      static_cast<unsigned long>(now_us), motorModeName(motor_mode),
      vq_command_v, electrical_angle_rad, open_loop_hz,
      encoder_status_valid ? 1 : 0, encoder_sample_valid ? 1 : 0,
      encoder_status.magnet_detected ? 1 : 0,
      static_cast<unsigned>(wheel_state.raw_count),
      static_cast<long long>(wheel_state.unwrapped_count), wheel_state.angle_rad,
      wheel_state.unwrapped_angle_rad, wheel_state.velocity_rad_s,
      wheel_state.instantaneous_velocity_rad_s,
      wheel_state.velocity_valid ? 1 : 0,
      static_cast<unsigned long>(encoder_read_errors));
}

bool initConsole() {
  uart_config_t config{};
  config.baud_rate = 115200;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  if (uart_param_config(UART_NUM_0, &config) != ESP_OK) {
    return false;
  }
  if (uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
    return false;
  }
  const esp_err_t result = uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
  return result == ESP_OK || result == ESP_ERR_INVALID_STATE;
}

bool initEncoderBus(i2c_master_bus_handle_t* bus) {
  i2c_master_bus_config_t config{};
  config.i2c_port = I2C_NUM_0;
  config.sda_io_num = static_cast<gpio_num_t>(triwhirl::board::kAs5600SdaGpio);
  config.scl_io_num = static_cast<gpio_num_t>(triwhirl::board::kAs5600SclGpio);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;
  return i2c_new_master_bus(&config, bus) == ESP_OK;
}

}  // namespace

extern "C" void app_main(void) {
  if (!initConsole()) {
    return;
  }

  i2c_master_bus_handle_t i2c_bus = nullptr;
  if (!initEncoderBus(&i2c_bus) ||
      !encoder.init(i2c_bus, triwhirl::board::kAs5600I2cAddress)) {
    consoleWrite("FATAL AS5600 I2C init failed\r\n");
    return;
  }

  if (!bridge.init(triwhirl::board::kMotorIn1Gpio,
                   triwhirl::board::kMotorIn2Gpio,
                   triwhirl::board::kMotorIn3Gpio, kPwmFrequencyHz,
                   triwhirl::board::kMotorBusNominalV)) {
    consoleWrite("FATAL MCPWM bridge init failed\r\n");
    return;
  }

  bridge.stopZeroVector();
  const std::uint32_t now_us = static_cast<std::uint32_t>(esp_timer_get_time());
  sampleEncoder(now_us);
  refreshEncoderHealth();
  last_motor_update_us = now_us;
  last_encoder_sample_us = now_us;
  last_encoder_health_us = now_us;
  last_telemetry_us = now_us;

  consoleWrite("TriWhirl motor runtime ready\r\n");
  consoleWrite("telemetry is off by default; use 'telemetry on' when streaming is needed\r\n");
  consoleWrite("telemetry_fields,t_us,mode,vq_v,e_angle_rad,e_hz,status_ok,sample_ok,mag,raw,unwrapped_count,angle_rad,unwrapped_rad,vel_rad_s,vel_inst_rad_s,vel_valid,read_errors\r\n");
  printStatus();
  printHelp();
  printPrompt();

  while (true) {
    const std::uint32_t loop_us = static_cast<std::uint32_t>(esp_timer_get_time());
    updateEncoder(loop_us);
    updateMotor(loop_us);
    pollConsole();
    emitTelemetry(loop_us);
    vTaskDelay(1);
  }
}
