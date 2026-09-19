#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "triwhirl/board.hpp"
#include "triwhirl/drivers/as5600.hpp"
#include "triwhirl/three_phase_field.hpp"
#include "triwhirl/wheel_kinematics.hpp"

namespace {

using triwhirl::WheelKinematics;
using triwhirl::WheelKinematicsState;
using triwhirl::board::kAs5600I2cAddress;
using triwhirl::board::kAs5600SclGpio;
using triwhirl::board::kAs5600SdaGpio;
using triwhirl::board::kBringupPhaseAmplitudeMaxV;
using triwhirl::board::kMotorBusNominalV;
using triwhirl::board::kMotorIn1Gpio;
using triwhirl::board::kMotorIn2Gpio;
using triwhirl::board::kMotorIn3Gpio;
using triwhirl::drivers::As5600;
using triwhirl::drivers::As5600Status;

constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kDriverVoltageLimitV = 3.0F;
constexpr float kMaxElectricalHz = 30.0F;
constexpr float kWheelVelocityFilterTauS = 0.01F;
constexpr std::uint32_t kEncoderSamplePeriodUs = 1000U;
constexpr std::uint32_t kEncoderHealthPeriodUs = 100000U;
constexpr std::uint32_t kTelemetryPeriodUs = 20000U;

As5600 encoder(Wire, kAs5600I2cAddress);
WheelKinematics wheel_kinematics(kWheelVelocityFilterTauS);
BLDCDriver3PWM driver(kMotorIn1Gpio, kMotorIn2Gpio, kMotorIn3Gpio);

As5600Status encoder_status{};
WheelKinematicsState wheel_state{};
bool encoder_status_valid = false;
bool encoder_sample_valid = false;
std::uint32_t encoder_read_errors = 0U;

bool field_enabled = false;
float field_amplitude_v = 0.0F;
float electrical_hz = 0.0F;
float electrical_angle_rad = 0.0F;
std::uint32_t last_field_update_us = 0U;
std::uint32_t last_encoder_sample_us = 0U;
std::uint32_t last_encoder_health_us = 0U;
std::uint32_t last_telemetry_us = 0U;

char command_line[96]{};
std::size_t command_length = 0U;

float clampFinite(const float value, const float low, const float high) {
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  return constrain(value, low, high);
}

bool refreshEncoderHealth() {
  As5600Status status{};
  if (!encoder.readStatus(&status)) {
    encoder_status = {};
    encoder_status_valid = false;
    return false;
  }

  encoder_status = status;
  encoder_status_valid = true;
  return true;
}

bool sampleEncoder() {
  std::uint16_t raw_count = 0U;
  if (!encoder.readRawAngle(&raw_count)) {
    encoder_sample_valid = false;
    ++encoder_read_errors;
    return false;
  }

  const std::uint32_t sample_time_us = micros();
  wheel_state = wheel_kinematics.update(raw_count, sample_time_us);
  encoder_sample_valid = true;
  return true;
}

bool encoderReadyForExcitation() {
  const bool status_ok = refreshEncoderHealth();
  const bool sample_ok = sampleEncoder();
  return status_ok && sample_ok && encoder_status.magnet_detected &&
         !encoder_status.magnet_too_weak &&
         !encoder_status.magnet_too_strong;
}

void updateEncoder(const std::uint32_t now_us) {
  if ((now_us - last_encoder_sample_us) >= kEncoderSamplePeriodUs) {
    last_encoder_sample_us = now_us;
    sampleEncoder();
  }

  if ((now_us - last_encoder_health_us) >= kEncoderHealthPeriodUs) {
    last_encoder_health_us = now_us;
    refreshEncoderHealth();
  }
}

void stopField() {
  field_enabled = false;
  field_amplitude_v = 0.0F;
  electrical_hz = 0.0F;
  driver.setPwm(0.0F, 0.0F, 0.0F);
}

void printHelp() {
  Serial.println("commands:");
  Serial.println("  field <electrical_hz> <amplitude_v>");
  Serial.println("  stop");
  Serial.println("  status");
  Serial.println("  help");
  Serial.println("notes:");
  Serial.println("  field is open-loop electrical excitation, not FOC");
  Serial.println("  field requires a healthy AS5600 magnet status and angle read");
  Serial.println("  amplitude is clamped to the bring-up ceiling");
}

void printStatus() {
  refreshEncoderHealth();

  Serial.printf(
      "status,enabled=%d,e_hz=%.6f,amp_v=%.6f,status_ok=%d,sample_ok=%d,mag=%d,ml=%d,mh=%d,raw=%u,unwrapped_count=%lld,angle_rad=%.6f,unwrapped_rad=%.6f,vel_rad_s=%.6f,vel_inst_rad_s=%.6f,vel_valid=%d,read_errors=%lu\n",
      field_enabled ? 1 : 0,
      electrical_hz,
      field_amplitude_v,
      encoder_status_valid ? 1 : 0,
      encoder_sample_valid ? 1 : 0,
      encoder_status.magnet_detected ? 1 : 0,
      encoder_status.magnet_too_weak ? 1 : 0,
      encoder_status.magnet_too_strong ? 1 : 0,
      static_cast<unsigned>(wheel_state.raw_count),
      static_cast<long long>(wheel_state.unwrapped_count),
      wheel_state.angle_rad,
      wheel_state.unwrapped_angle_rad,
      wheel_state.velocity_rad_s,
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
    Serial.println("OK stop");
    return;
  }

  if (std::strcmp(command, "status") == 0) {
    printStatus();
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
      Serial.println("ERR usage: field <electrical_hz> <amplitude_v>");
      return;
    }

    if (!encoderReadyForExcitation()) {
      stopField();
      Serial.println("ERR AS5600/magnet not ready; field remains disabled");
      return;
    }

    const float requested_hz = std::strtof(hz_token, nullptr);
    const float requested_amplitude = std::strtof(amplitude_token, nullptr);

    electrical_hz = clampFinite(requested_hz, -kMaxElectricalHz, kMaxElectricalHz);
    field_amplitude_v = clampFinite(
        requested_amplitude, 0.0F, kBringupPhaseAmplitudeMaxV);

    if (field_amplitude_v <= 0.0F || electrical_hz == 0.0F) {
      stopField();
      Serial.println("OK field disabled");
    } else {
      field_enabled = true;
      Serial.printf("OK field e_hz=%.6f amp_v=%.6f\n",
                    electrical_hz,
                    field_amplitude_v);
    }
    return;
  }

  Serial.println("ERR unknown command");
}

void pollConsole() {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      return;
    }

    const char c = static_cast<char>(value);
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
      Serial.println("ERR command too long");
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

  const auto phase = triwhirl::makeRotatingField(
      electrical_angle_rad, field_amplitude_v, kDriverVoltageLimitV);
  driver.setPwm(phase.a, phase.b, phase.c);
}

void emitTelemetry(const std::uint32_t now_us) {
  if ((now_us - last_telemetry_us) < kTelemetryPeriodUs) {
    return;
  }
  last_telemetry_us = now_us;

  Serial.printf(
      "telemetry,%lu,%d,%.6f,%.6f,%d,%d,%d,%u,%lld,%.6f,%.6f,%.6f,%.6f,%d,%lu\n",
      static_cast<unsigned long>(now_us),
      field_enabled ? 1 : 0,
      electrical_hz,
      field_amplitude_v,
      encoder_status_valid ? 1 : 0,
      encoder_sample_valid ? 1 : 0,
      encoder_status.magnet_detected ? 1 : 0,
      static_cast<unsigned>(wheel_state.raw_count),
      static_cast<long long>(wheel_state.unwrapped_count),
      wheel_state.angle_rad,
      wheel_state.unwrapped_angle_rad,
      wheel_state.velocity_rad_s,
      wheel_state.instantaneous_velocity_rad_s,
      wheel_state.velocity_valid ? 1 : 0,
      static_cast<unsigned long>(encoder_read_errors));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);

  if (!Wire.begin(kAs5600SdaGpio, kAs5600SclGpio, 400000U)) {
    Serial.println("FATAL AS5600 I2C bus init failed");
    while (true) {
      delay(1000);
    }
  }

  refreshEncoderHealth();
  sampleEncoder();

  driver.voltage_power_supply = kMotorBusNominalV;
  driver.voltage_limit = kDriverVoltageLimitV;
  if (!driver.init()) {
    Serial.println("FATAL motor driver init failed");
    while (true) {
      delay(1000);
    }
  }

  stopField();
  const std::uint32_t now_us = micros();
  last_field_update_us = now_us;
  last_encoder_sample_us = now_us;
  last_encoder_health_us = now_us;
  last_telemetry_us = now_us;

  Serial.println("TriWhirl motor bring-up ready");
  Serial.println(
      "telemetry_fields,t_us,field_enabled,e_hz,amp_v,status_ok,sample_ok,mag,raw,unwrapped_count,angle_rad,unwrapped_rad,vel_rad_s,vel_inst_rad_s,vel_valid,read_errors");
  printStatus();
  printHelp();
}

void loop() {
  const std::uint32_t now_us = micros();
  updateEncoder(now_us);
  updateField(now_us);
  pollConsole();
  emitTelemetry(now_us);
}
