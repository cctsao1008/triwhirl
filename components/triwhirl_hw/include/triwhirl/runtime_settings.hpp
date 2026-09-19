#pragma once

namespace triwhirl {
namespace settings {

struct MotorSettings {
  int pole_pairs = 0;
  int sensor_direction = 1;
  float electrical_offset_rad = 0.0F;
};

struct ImuMapSettings {
  int accel_sin_axis = 0;
  int accel_cos_axis = 1;
  int gyro_axis = 2;
  int accel_sin_sign = 1;
  int accel_cos_sign = 1;
  int gyro_sign = 1;
};

// Initializes the ESP-IDF NVS backend. Safe to call even when another
// subsystem (for example NimBLE) already initialized NVS.
bool init();

bool loadMotor(MotorSettings* settings);
bool saveMotor(const MotorSettings& settings);
bool clearMotor();

bool loadImuMap(ImuMapSettings* settings);
bool saveImuMap(const ImuMapSettings& settings);
bool clearImuMap();

}  // namespace settings
}  // namespace triwhirl
