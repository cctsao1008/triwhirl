#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "triwhirl/board.hpp"
#include "triwhirl/three_phase_field.hpp"

namespace {

using triwhirl::board::kAs5600SclGpio;
using triwhirl::board::kAs5600SdaGpio;
using triwhirl::board::kBringupPhaseAmplitudeMaxV;
using triwhirl::board::kMotorBusNominalV;
using triwhirl::board::kMotorIn1Gpio;
using triwhirl::board::kMotorIn2Gpio;
using triwhirl::board::kMotorIn3Gpio;

constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kDriverVoltageLimitV = 3.0F;
constexpr float kMaxElectricalHz = 30.0F;
constexpr std::uint32_t kTelemetryPeriodUs = 20000;

MagneticSensorI2C sensor(AS5600_I2C);
BLDCDriver3PWM driver(kMotorIn1Gpio, kMotorIn2Gpio, kMotorIn3Gpio);

bool field_enabled = false;
bool as5600_present = false;
bool as5600_magnet_detected = false;
bool as5600_magnet_too_weak = false;
bool as5600_magnet_too_strong = false;
float field_amplitude_v = 0.0F;
float electrical_hz = 0.0F;
float electrical_angle_rad = 0.0F;
std::uint32_t last_update_us = 0;
std::uint32_t last_telemetry_us = 0;

char command_line[96]{};
std::size_t command_length = 0;

constexpr std::uint8_t kAs5600Address = 0x36;
constexpr std::uint8_t kAs5600StatusRegister = 0x0B;
constexpr std::uint8_t kAs5600StatusMagnetDetected = 1U << 5;
constexpr std::uint8_t kAs5600StatusMagnetTooWeak = 1U << 4;
constexpr std::uint8_t kAs5600StatusMagnetTooStrong = 1U << 3;

bool readAs5600Register(const std::uint8_t reg, std::uint8_t* value) {
  Wire.beginTransmission(kAs5600Address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(kAs5600Address, static_cast<std::uint8_t>(1)) != 1) {
    return false;
  }

  *value = static_cast<std::uint8_t>(Wire.read());
  return true;
}

void updateAs5600Health() {
  std::uint8_t status = 0;
  as5600_present = readAs5600Register(kAs5600StatusRegister, &status);
  if (!as5600_present) {
    as5600_magnet_detected = false;
    as5600_magnet_too_weak = false;
    as5600_magnet_too_strong = false;
    return;
  }

  as5600_magnet_detected = (status & kAs5600StatusMagnetDetected) != 0;
  as5600_magnet_too_weak = (status & kAs5600StatusMagnetTooWeak) != 0;
  as5600_magnet_too_strong = (status & kAs5600StatusMagnetTooStrong) != 0;
}

bool as5600HealthyForMotorExcitation() {
  return as5600_present && as5600_magnet_detected &&
         !as5600_magnet_too_weak && !as5600_magnet_too_strong;
}

float clampFinite(const float value, const float low, const float high) {
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  return constrain(value, low, high);
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
  Serial.println("  amplitude is clamped to the bring-up ceiling");
  Serial.println("  field requires a healthy AS5600 magnetic reading");
}

void printStatus() {
  updateAs5600Health();
  sensor.update();
  Serial.printf(
      "status,enabled=%d,e_hz=%.6f,amp_v=%.6f,angle_rad=%.6f,vel_rad_s=%.6f,as5600=%d,mag=%d,ml=%d,mh=%d\n",
      field_enabled ? 1 : 0,
      electrical_hz,
      field_amplitude_v,
      sensor.getAngle(),
      sensor.getVelocity(),
      as5600_present ? 1 : 0,
      as5600_magnet_detected ? 1 : 0,
      as5600_magnet_too_weak ? 1 : 0,
      as5600_magnet_too_strong ? 1 : 0);
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

    updateAs5600Health();
    if (!as5600HealthyForMotorExcitation()) {
      stopField();
      Serial.println("ERR AS5600 not healthy; motor excitation refused");
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
      command_length = 0;
      continue;
    }

    if (command_length + 1 < sizeof(command_line)) {
      command_line[command_length++] = c;
    } else {
      command_length = 0;
      Serial.println("ERR command too long");
    }
  }
}

void updateField(const std::uint32_t now_us) {
  const std::uint32_t elapsed_us = now_us - last_update_us;
  last_update_us = now_us;

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

  sensor.update();
  Serial.printf("telemetry,%lu,%d,%.6f,%.6f,%.6f,%.6f\n",
                static_cast<unsigned long>(now_us),
                field_enabled ? 1 : 0,
                electrical_hz,
                field_amplitude_v,
                sensor.getAngle(),
                sensor.getVelocity());
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);

  // ESP32 requires begin() to bind non-default I2C pins.
  Wire.begin(kAs5600SdaGpio, kAs5600SclGpio, 400000U);
  sensor.init(&Wire);
  updateAs5600Health();

  driver.voltage_power_supply = kMotorBusNominalV;
  driver.voltage_limit = kDriverVoltageLimitV;
  if (!driver.init()) {
    Serial.println("FATAL motor driver init failed");
    while (true) {
      delay(1000);
    }
  }

  stopField();
  last_update_us = micros();
  last_telemetry_us = last_update_us;

  Serial.println("TriWhirl motor bring-up ready");
  printStatus();
  printHelp();
}

void loop() {
  const std::uint32_t now_us = micros();
  updateField(now_us);
  pollConsole();
  emitTelemetry(now_us);
}
