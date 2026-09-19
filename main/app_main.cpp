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
#include "triwhirl/three_phase_field.hpp"
#include "triwhirl/wheel_kinematics.hpp"

namespace {

using triwhirl::WheelKinematics;
using triwhirl::WheelKinematicsState;
using triwhirl::drivers::As5600;
using triwhirl::drivers::As5600Status;
using triwhirl::motor::ThreePwmBridge;

constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kDriverVoltageLimitV = 3.0F;
constexpr float kMaxElectricalHz = 30.0F;
constexpr float kWheelVelocityFilterTauS = 0.01F;
constexpr std::uint32_t kEncoderSamplePeriodUs = 1000U;
constexpr std::uint32_t kEncoderHealthPeriodUs = 100000U;
constexpr std::uint32_t kTelemetryPeriodUs = 20000U;
constexpr std::uint32_t kPwmFrequencyHz = 25000U;

As5600 encoder;
WheelKinematics wheel_kinematics(kWheelVelocityFilterTauS);
ThreePwmBridge bridge;

As5600Status encoder_status{};
WheelKinematicsState wheel_state{};
bool encoder_status_valid = false;
bool encoder_sample_valid = false;
std::uint32_t encoder_read_errors = 0U;

bool field_enabled = false;
bool telemetry_enabled = false;
float field_amplitude_v = 0.0F;
float electrical_hz = 0.0F;
float electrical_angle_rad = 0.0F;
std::uint32_t last_field_update_us = 0U;
std::uint32_t last_encoder_sample_us = 0U;
std::uint32_t last_encoder_health_us = 0U;
std::uint32_t last_telemetry_us = 0U;

char command_line[96]{};
std::size_t command_length = 0U;

void consoleWrite(const char* text) {
  if (text == nullptr) {
    return;
  }
  uart_write_bytes(UART_NUM_0, text, std::strlen(text));
}

void consolePrintf(const char* format, ...) {
  char buffer[384];
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

bool encoderHealthy() {
  return encoder_status_valid && encoder_sample_valid &&
         encoder_status.magnet_detected && !encoder_status.magnet_too_weak &&
         !encoder_status.magnet_too_strong;
}

void stopField() {
  field_enabled = false;
  field_amplitude_v = 0.0F;
  electrical_hz = 0.0F;
  bridge.stopZeroVector();
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
  if (field_enabled && !encoderHealthy()) {
    stopField();
    consoleWrite("ERR encoder health lost; field stopped\r\n");
  }
}

void printHelp() {
  consoleWrite("commands:\r\n");
  consoleWrite("  field <electrical_hz> <amplitude_v>\r\n");
  consoleWrite("  stop\r\n");
  consoleWrite("  status\r\n");
  consoleWrite("  telemetry [on|off]\r\n");
  consoleWrite("  help\r\n");
}

void printStatus() {
  refreshEncoderHealth();
  consolePrintf(
      "status,enabled=%d,telemetry=%d,e_hz=%.6f,amp_v=%.6f,status_ok=%d,sample_ok=%d,mag=%d,ml=%d,mh=%d,raw=%u,unwrapped_count=%lld,angle_rad=%.6f,unwrapped_rad=%.6f,vel_rad_s=%.6f,vel_inst_rad_s=%.6f,vel_valid=%d,read_errors=%lu\r\n",
      field_enabled ? 1 : 0, telemetry_enabled ? 1 : 0, electrical_hz,
      field_amplitude_v, encoder_status_valid ? 1 : 0,
      encoder_sample_valid ? 1 : 0,
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

void handleCommand(char* line) {
  char* command = std::strtok(line, " \t");
  if (command == nullptr) {
    return;
  }
  if (std::strcmp(command, "stop") == 0) {
    stopField();
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
    refreshEncoderHealth();
    if (!encoderHealthy()) {
      stopField();
      consoleWrite("ERR AS5600/magnet not ready; field remains stopped\r\n");
      return;
    }
    electrical_hz = clampFinite(std::strtof(hz_token, nullptr), -kMaxElectricalHz,
                                kMaxElectricalHz);
    field_amplitude_v = clampFinite(std::strtof(amplitude_token, nullptr), 0.0F,
                                    triwhirl::board::kBringupPhaseAmplitudeMaxV);
    if (field_amplitude_v <= 0.0F || electrical_hz == 0.0F) {
      stopField();
      consoleWrite("OK field stopped\r\n");
    } else {
      field_enabled = true;
      consolePrintf("OK field e_hz=%.6f amp_v=%.6f\r\n", electrical_hz,
                    field_amplitude_v);
    }
    return;
  }
  consoleWrite("ERR unknown command\r\n");
}

void pollConsole() {
  std::uint8_t input[32];
  const int received = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
  for (int i = 0; i < received; ++i) {
    const char c = static_cast<char>(input[i]);
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      command_line[command_length] = '\0';
      handleCommand(command_line);
      command_length = 0U;
      continue;
    }
    if (command_length + 1U < sizeof(command_line)) {
      command_line[command_length++] = c;
    } else {
      command_length = 0U;
      consoleWrite("ERR command too long\r\n");
    }
  }
}

void updateField(const std::uint32_t now_us) {
  const std::uint32_t elapsed_us = now_us - last_field_update_us;
  last_field_update_us = now_us;
  if (!field_enabled) {
    return;
  }
  const float dt = static_cast<float>(elapsed_us) * 1.0e-6F;
  electrical_angle_rad += kTwoPi * electrical_hz * dt;
  electrical_angle_rad = std::fmod(electrical_angle_rad, kTwoPi);
  if (electrical_angle_rad < 0.0F) {
    electrical_angle_rad += kTwoPi;
  }
  const triwhirl::PhaseVoltages phase = triwhirl::makeRotatingField(
      electrical_angle_rad, field_amplitude_v, kDriverVoltageLimitV);
  bridge.setPhaseVoltages(phase.a, phase.b, phase.c);
}

void emitTelemetry(const std::uint32_t now_us) {
  if (!telemetry_enabled ||
      (now_us - last_telemetry_us) < kTelemetryPeriodUs) {
    return;
  }
  last_telemetry_us = now_us;
  consolePrintf(
      "telemetry,%lu,%d,%.6f,%.6f,%d,%d,%d,%u,%lld,%.6f,%.6f,%.6f,%.6f,%d,%lu\r\n",
      static_cast<unsigned long>(now_us), field_enabled ? 1 : 0, electrical_hz,
      field_amplitude_v, encoder_status_valid ? 1 : 0,
      encoder_sample_valid ? 1 : 0,
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
  last_field_update_us = now_us;
  last_encoder_sample_us = now_us;
  last_encoder_health_us = now_us;
  last_telemetry_us = now_us;

  consoleWrite("TriWhirl native ESP-IDF motor bring-up ready\r\n");
  consoleWrite("telemetry is off by default; use 'telemetry on' when streaming is needed\r\n");
  consoleWrite("telemetry_fields,t_us,field_enabled,e_hz,amp_v,status_ok,sample_ok,mag,raw,unwrapped_count,angle_rad,unwrapped_rad,vel_rad_s,vel_inst_rad_s,vel_valid,read_errors\r\n");
  printStatus();
  printHelp();

  while (true) {
    const std::uint32_t loop_us = static_cast<std::uint32_t>(esp_timer_get_time());
    updateEncoder(loop_us);
    updateField(loop_us);
    pollConsole();
    emitTelemetry(loop_us);
    vTaskDelay(1);
  }
}
