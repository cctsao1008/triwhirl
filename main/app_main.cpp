#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <limits>

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "triwhirl/attitude_estimator.hpp"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/board.hpp"
#include "triwhirl/drivers/as5600.hpp"
#include "triwhirl/drivers/mpu6050.hpp"
#include "triwhirl/motor/three_pwm_bridge.hpp"
#include "triwhirl/safety.hpp"
#include "triwhirl/voltage_mode_foc.hpp"
#include "triwhirl/wheel_kinematics.hpp"

namespace {

using triwhirl::AttitudeEstimate;
using triwhirl::MotorElectricalConfig;
using triwhirl::PhaseVoltages;
using triwhirl::PlanarAttitudeEstimator;
using triwhirl::SafetyFault;
using triwhirl::SafetyLatch;
using triwhirl::WheelKinematics;
using triwhirl::WheelKinematicsState;
using triwhirl::drivers::As5600;
using triwhirl::drivers::As5600Status;
using triwhirl::drivers::Mpu6050;
using triwhirl::drivers::Mpu6050Sample;
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
constexpr std::uint32_t kControlPeriodUs = 1000U;
constexpr std::uint32_t kHardControlPeriodUs = 20U * kControlPeriodUs;
constexpr std::uint32_t kEncoderSamplePeriodUs = 1000U;
constexpr std::uint32_t kEncoderHealthPeriodUs = 100000U;
constexpr std::uint32_t kImuSamplePeriodUs = 1000U;
constexpr std::uint32_t kDefaultGyroCalibrationSamples = 500U;
constexpr std::uint32_t kTelemetryPeriodUs = 20000U;
constexpr std::uint32_t kPwmFrequencyHz = 25000U;
constexpr std::size_t kConsoleTxBufferBytes = 8192U;

struct CommandInputState {
  char line[128]{};
  std::size_t length = 0U;
};

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

struct GyroCalibrationState {
  bool active = false;
  std::uint32_t target_samples = kDefaultGyroCalibrationSamples;
  std::uint32_t collected_samples = 0U;
  double sum_rad_s[3]{};
};

struct ImuPlanarMap {
  int accel_sin_axis = 0;
  int accel_cos_axis = 1;
  int gyro_axis = 2;
  int accel_sin_sign = 1;
  int accel_cos_sign = 1;
  int gyro_sign = 1;
};

struct ControlTimingStats {
  std::uint64_t iterations = 0U;
  std::uint64_t overruns = 0U;
  std::uint64_t late_periods = 0U;
  std::uint32_t last_exec_us = 0U;
  std::uint32_t max_exec_us = 0U;
  std::uint32_t min_period_us = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t max_period_us = 0U;
  std::int64_t previous_start_us = 0;
};

As5600 encoder;
Mpu6050 imu;
WheelKinematics wheel_kinematics(kWheelVelocityFilterTauS);
PlanarAttitudeEstimator attitude_estimator;
ThreePwmBridge bridge;
SafetyLatch safety_latch;

As5600Status encoder_status{};
WheelKinematicsState wheel_state{};
bool encoder_status_valid = false;
bool encoder_sample_valid = false;
std::uint32_t encoder_read_errors = 0U;

Mpu6050Sample imu_sample{};
bool imu_ready = false;
bool imu_sample_valid = false;
bool gyro_bias_valid = false;
float gyro_bias_rad_s[3]{};
std::uint32_t imu_read_errors = 0U;
GyroCalibrationState gyro_calibration{};
ImuPlanarMap imu_map{};
AttitudeEstimate attitude_state{};
bool attitude_initialized = false;
std::uint32_t last_attitude_update_us = 0U;

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
std::uint32_t last_imu_sample_us = 0U;
std::uint32_t last_telemetry_us = 0U;

ControlTimingStats timing_stats{};
StreamBufferHandle_t console_tx_stream = nullptr;
std::uint32_t console_tx_dropped_bytes = 0U;
CommandInputState uart_command_input{};
CommandInputState ble_command_input{};

void consoleWriteBytes(const char* data, const std::size_t length) {
  if (data == nullptr || length == 0U) {
    return;
  }

  if (console_tx_stream == nullptr) {
    uart_write_bytes(UART_NUM_0, data, length);
  } else {
    const std::size_t sent = xStreamBufferSend(console_tx_stream, data, length, 0);
    if (sent < length) {
      console_tx_dropped_bytes += static_cast<std::uint32_t>(length - sent);
    }
  }

  triwhirl::ble::write(reinterpret_cast<const std::uint8_t*>(data), length);
}

void consoleWrite(const char* text) {
  if (text == nullptr) {
    return;
  }
  consoleWriteBytes(text, std::strlen(text));
}

void consolePrintf(const char* format, ...) {
  char buffer[768];
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
  consoleWriteBytes(buffer, count);
}

void consoleTxTask(void*) {
  std::uint8_t buffer[256];
  while (true) {
    const std::size_t received = xStreamBufferReceive(
        console_tx_stream, buffer, sizeof(buffer), portMAX_DELAY);
    if (received > 0U) {
      uart_write_bytes(UART_NUM_0, buffer, received);
    }
  }
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

bool validAxis(const int axis) {
  return axis >= 0 && axis <= 2;
}

bool validSign(const int sign) {
  return sign == 1 || sign == -1;
}

bool validImuMap(const ImuPlanarMap& map) {
  return validAxis(map.accel_sin_axis) && validAxis(map.accel_cos_axis) &&
         validAxis(map.gyro_axis) &&
         map.accel_sin_axis != map.accel_cos_axis &&
         validSign(map.accel_sin_sign) && validSign(map.accel_cos_sign) &&
         validSign(map.gyro_sign);
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

float correctedGyro(const int axis) {
  if (axis < 0 || axis > 2 || !imu_sample_valid) {
    return 0.0F;
  }
  return imu_sample.gyro_rad_s[axis] -
         (gyro_bias_valid ? gyro_bias_rad_s[axis] : 0.0F);
}

float mappedAccelSin() {
  return static_cast<float>(imu_map.accel_sin_sign) *
         imu_sample.accel_mps2[imu_map.accel_sin_axis];
}

float mappedAccelCos() {
  return static_cast<float>(imu_map.accel_cos_sign) *
         imu_sample.accel_mps2[imu_map.accel_cos_axis];
}

float mappedGyro() {
  return static_cast<float>(imu_map.gyro_sign) *
         correctedGyro(imu_map.gyro_axis);
}

void resetAttitudeFromAccel() {
  if (!imu_sample_valid || !validImuMap(imu_map)) {
    attitude_initialized = false;
    attitude_state = {};
    return;
  }
  const float initial_angle = std::atan2(mappedAccelSin(), mappedAccelCos());
  attitude_estimator.reset(initial_angle, 0.0F);
  attitude_state = attitude_estimator.state();
  attitude_initialized = true;
  last_attitude_update_us = static_cast<std::uint32_t>(esp_timer_get_time());
}

void updateAttitude(const std::uint32_t now_us) {
  if (!imu_sample_valid || !gyro_bias_valid || !validImuMap(imu_map)) {
    return;
  }

  if (!attitude_initialized) {
    resetAttitudeFromAccel();
    last_attitude_update_us = now_us;
    return;
  }

  const std::uint32_t elapsed_us = now_us - last_attitude_update_us;
  if (elapsed_us == 0U) {
    return;
  }
  last_attitude_update_us = now_us;
  const float dt_s = static_cast<float>(elapsed_us) * 1.0e-6F;
  attitude_state = attitude_estimator.update(
      mappedAccelSin(), mappedAccelCos(), mappedGyro(), dt_s, true);
}

void startGyroCalibration(std::uint32_t samples) {
  if (!imu_ready) {
    consoleWrite("ERR imu unavailable\r\n");
    return;
  }
  if (samples < 50U) {
    samples = 50U;
  } else if (samples > 5000U) {
    samples = 5000U;
  }
  gyro_calibration = {};
  gyro_calibration.active = true;
  gyro_calibration.target_samples = samples;
  gyro_bias_valid = false;
  attitude_initialized = false;
  attitude_state = {};
  consolePrintf("OK imu gyro calibration started samples=%lu\r\n",
                static_cast<unsigned long>(samples));
}

bool sampleImu() {
  Mpu6050Sample sample{};
  if (!imu_ready || !imu.readSample(&sample)) {
    imu_sample_valid = false;
    if (imu_ready) {
      ++imu_read_errors;
    }
    return false;
  }

  imu_sample = sample;
  imu_sample_valid = true;

  if (gyro_calibration.active) {
    for (int axis = 0; axis < 3; ++axis) {
      gyro_calibration.sum_rad_s[axis] +=
          static_cast<double>(sample.gyro_rad_s[axis]);
    }
    ++gyro_calibration.collected_samples;
    if (gyro_calibration.collected_samples >= gyro_calibration.target_samples) {
      const double denominator =
          static_cast<double>(gyro_calibration.collected_samples);
      for (int axis = 0; axis < 3; ++axis) {
        gyro_bias_rad_s[axis] = static_cast<float>(
            gyro_calibration.sum_rad_s[axis] / denominator);
      }
      gyro_calibration.active = false;
      gyro_bias_valid = true;
      consolePrintf(
          "OK imu gyro calibration bx=%.6f by=%.6f bz=%.6f rad_s\r\n",
          gyro_bias_rad_s[0], gyro_bias_rad_s[1], gyro_bias_rad_s[2]);
    }
  }

  return true;
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

void updateImu(const std::uint32_t now_us) {
  if ((now_us - last_imu_sample_us) < kImuSamplePeriodUs) {
    return;
  }
  last_imu_sample_us = now_us;
  if (sampleImu()) {
    updateAttitude(now_us);
  }
}

void stopMotor() {
  motor_mode = MotorMode::kStopped;
  calibration.stage = CalibrationStage::kIdle;
  open_loop_amplitude_v = 0.0F;
  open_loop_hz = 0.0F;
  vq_command_v = 0.0F;
  bridge.stopZeroVector();
}

bool motorActive() {
  return motor_mode != MotorMode::kStopped;
}

bool hasFault(const SafetyFault fault) {
  return (safety_latch.mask() & triwhirl::safetyFaultMask(fault)) != 0U;
}

void tripFault(const SafetyFault fault) {
  const std::uint32_t before = safety_latch.mask();
  safety_latch.trip(fault);
  stopMotor();
  if ((before & triwhirl::safetyFaultMask(fault)) == 0U) {
    consolePrintf("FAULT,code=%s,mask=0x%08lx\r\n",
                  triwhirl::safetyFaultName(fault),
                  static_cast<unsigned long>(safety_latch.mask()));
  }
}

bool motorNumericsHealthy() {
  if (!std::isfinite(wheel_state.angle_rad) ||
      !std::isfinite(wheel_state.unwrapped_angle_rad) ||
      !std::isfinite(open_loop_hz) ||
      !std::isfinite(open_loop_amplitude_v) ||
      !std::isfinite(open_loop_angle_rad) ||
      !std::isfinite(vq_command_v) ||
      !std::isfinite(electrical_angle_rad)) {
    return false;
  }
  if (wheel_state.velocity_valid &&
      (!std::isfinite(wheel_state.velocity_rad_s) ||
       !std::isfinite(wheel_state.instantaneous_velocity_rad_s))) {
    return false;
  }
  if (attitude_state.valid &&
      (!std::isfinite(attitude_state.angle_rad) ||
       !std::isfinite(attitude_state.rate_rad_s))) {
    return false;
  }
  return true;
}

bool motorStartAllowed() {
  if (safety_latch.faulted()) {
    consolePrintf("ERR safety fault latched first=%s mask=0x%08lx; use 'fault status'\r\n",
                  triwhirl::safetyFaultName(safety_latch.firstFault()),
                  static_cast<unsigned long>(safety_latch.mask()));
    return false;
  }
  if (!encoder_sample_valid) {
    consoleWrite("ERR encoder read unavailable\r\n");
    return false;
  }
  if (!motorNumericsHealthy()) {
    consoleWrite("ERR invalid runtime numeric state\r\n");
    return false;
  }
  return true;
}

bool faultClearReady() {
  if (motorActive() || !motorNumericsHealthy()) {
    return false;
  }
  if (hasFault(SafetyFault::kEncoderUnavailable) && !encoder_sample_valid) {
    return false;
  }
  if (hasFault(SafetyFault::kImuUnavailable) && !imu_sample_valid) {
    return false;
  }
  return true;
}

void evaluateSafety(const std::int64_t start_us) {
  if (!motorActive() || safety_latch.faulted()) {
    return;
  }

  if (!encoder_sample_valid) {
    tripFault(SafetyFault::kEncoderUnavailable);
    return;
  }

  if (!motorNumericsHealthy()) {
    tripFault(SafetyFault::kInvalidNumeric);
    return;
  }

  if (motor_mode == MotorMode::kFoc && !motor_config_valid) {
    tripFault(SafetyFault::kCalibration);
    return;
  }

  if (timing_stats.previous_start_us != 0 &&
      start_us > timing_stats.previous_start_us) {
    const std::uint64_t period_us = static_cast<std::uint64_t>(
        start_us - timing_stats.previous_start_us);
    if (period_us > kHardControlPeriodUs) {
      tripFault(SafetyFault::kControlTiming);
    }
  }
}

bool applyDq(const float electrical_angle, const float vd_v, const float vq_v) {
  if (!std::isfinite(electrical_angle) || !std::isfinite(vd_v) ||
      !std::isfinite(vq_v)) {
    tripFault(SafetyFault::kInvalidNumeric);
    return false;
  }

  const PhaseVoltages phase = triwhirl::makeDqVoltage(
      electrical_angle, vd_v, vq_v, triwhirl::board::kMotorBusNominalV,
      kMotorVectorLimitV);
  if (!bridge.setPhaseVoltages(phase.a, phase.b, phase.c)) {
    tripFault(SafetyFault::kActuator);
    return false;
  }
  return true;
}

void finishCalibration() {
  const float delta_mechanical =
      wheel_state.unwrapped_angle_rad - calibration.start_mechanical_rad;
  const float total_electrical = calibration.electrical_turns * kTwoPi;
  const float mechanical_travel = std::fabs(delta_mechanical);

  if (!(mechanical_travel > 0.05F) || !std::isfinite(mechanical_travel)) {
    tripFault(SafetyFault::kCalibration);
    consoleWrite("ERR motor calibration: no usable mechanical motion\r\n");
    return;
  }

  const float pole_pairs_estimate = total_electrical / mechanical_travel;
  const int pole_pairs = static_cast<int>(std::lround(pole_pairs_estimate));
  if (pole_pairs < 1 || pole_pairs > 64 ||
      std::fabs(pole_pairs_estimate - static_cast<float>(pole_pairs)) > 0.45F) {
    tripFault(SafetyFault::kCalibration);
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

  if (!motor_config_valid) {
    tripFault(SafetyFault::kCalibration);
    consoleWrite("ERR motor calibration: generated configuration is invalid\r\n");
    return;
  }

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
    tripFault(SafetyFault::kEncoderUnavailable);
    consoleWrite("ERR motor calibration: encoder read unavailable\r\n");
    return;
  }

  if (calibration.stage == CalibrationStage::kAlign) {
    calibration.commanded_electrical_rad = 0.0F;
    if (!applyDq(0.0F, calibration.amplitude_v, 0.0F)) {
      return;
    }
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
    if (!applyDq(calibration.commanded_electrical_rad,
                 calibration.amplitude_v, 0.0F)) {
      return;
    }
    return;
  }

  if (calibration.stage == CalibrationStage::kSettle) {
    if (!applyDq(calibration.commanded_electrical_rad,
                 calibration.amplitude_v, 0.0F)) {
      return;
    }
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
    if (!motor_config_valid) {
      tripFault(SafetyFault::kCalibration);
      consoleWrite("ERR FOC stopped: motor configuration unavailable\r\n");
      return;
    }
    if (!encoder_sample_valid) {
      tripFault(SafetyFault::kEncoderUnavailable);
      consoleWrite("ERR FOC stopped: encoder unavailable\r\n");
      return;
    }
    electrical_angle_rad = triwhirl::electricalAngleFromMechanical(
        wheel_state.unwrapped_angle_rad, motor_config);
    applyDq(electrical_angle_rad, 0.0F, vq_command_v);
  }
}

void printBleStatus() {
  consolePrintf(
      "ble,connected=%d,subscribed=%d,rx_drop_bytes=%lu,tx_drop_bytes=%lu\r\n",
      triwhirl::ble::connected() ? 1 : 0,
      triwhirl::ble::subscribed() ? 1 : 0,
      static_cast<unsigned long>(triwhirl::ble::rxDroppedBytes()),
      static_cast<unsigned long>(triwhirl::ble::txDroppedBytes()));
}

void printFaultStatus() {
  consolePrintf("fault,latched=%d,mask=0x%08lx,first=%s\r\n",
                safety_latch.faulted() ? 1 : 0,
                static_cast<unsigned long>(safety_latch.mask()),
                triwhirl::safetyFaultName(safety_latch.firstFault()));
}

void printTimingStatus() {
  const std::uint32_t min_period =
      timing_stats.iterations > 1U ? timing_stats.min_period_us : 0U;
  consolePrintf(
      "timing,target_us=%lu,hard_period_us=%lu,iterations=%llu,last_exec_us=%lu,max_exec_us=%lu,min_period_us=%lu,max_period_us=%lu,overruns=%llu,late_periods=%llu,uart_tx_drop_bytes=%lu,ble_rx_drop_bytes=%lu,ble_tx_drop_bytes=%lu\r\n",
      static_cast<unsigned long>(kControlPeriodUs),
      static_cast<unsigned long>(kHardControlPeriodUs),
      static_cast<unsigned long long>(timing_stats.iterations),
      static_cast<unsigned long>(timing_stats.last_exec_us),
      static_cast<unsigned long>(timing_stats.max_exec_us),
      static_cast<unsigned long>(min_period),
      static_cast<unsigned long>(timing_stats.max_period_us),
      static_cast<unsigned long long>(timing_stats.overruns),
      static_cast<unsigned long long>(timing_stats.late_periods),
      static_cast<unsigned long>(console_tx_dropped_bytes),
      static_cast<unsigned long>(triwhirl::ble::rxDroppedBytes()),
      static_cast<unsigned long>(triwhirl::ble::txDroppedBytes()));
}

void resetTimingStats() {
  timing_stats = {};
  console_tx_dropped_bytes = 0U;
}

void printHelp() {
  consoleWrite("commands:\r\n");
  consoleWrite("  motor calibrate [amplitude_v] [electrical_hz] [turns]\r\n");
  consoleWrite("  motor config <pole_pairs> <sensor_dir> <offset_rad>\r\n");
  consoleWrite("  motor vq <volts>\r\n");
  consoleWrite("  motor status\r\n");
  consoleWrite("  motor stop\r\n");
  consoleWrite("  imu status\r\n");
  consoleWrite("  imu calibrate [samples]\r\n");
  consoleWrite("  imu map <sin_axis> <cos_axis> <gyro_axis> <sin_sign> <cos_sign> <gyro_sign>\r\n");
  consoleWrite("  attitude status\r\n");
  consoleWrite("  attitude reset [angle_rad]\r\n");
  consoleWrite("  timing status\r\n");
  consoleWrite("  timing reset\r\n");
  consoleWrite("  fault status\r\n");
  consoleWrite("  fault clear\r\n");
  consoleWrite("  ble status\r\n");
  consoleWrite("  field <electrical_hz> <amplitude_v>\r\n");
  consoleWrite("  stop\r\n");
  consoleWrite("  status\r\n");
  consoleWrite("  telemetry [on|off]\r\n");
  consoleWrite("  help\r\n");
}

void printStatus() {
  refreshEncoderHealth();
  consolePrintf(
      "status,mode=%s,telemetry=%d,vq_v=%.6f,e_hz=%.6f,amp_v=%.6f,config=%d,pole_pairs=%d,sensor_dir=%d,offset_rad=%.6f,e_angle_rad=%.6f,status_ok=%d,sample_ok=%d,mag=%d,ml=%d,mh=%d,raw=%u,unwrapped_count=%lld,angle_rad=%.6f,unwrapped_rad=%.6f,vel_rad_s=%.6f,vel_inst_rad_s=%.6f,vel_valid=%d,read_errors=%lu,imu_ok=%d,attitude_ok=%d,theta_rad=%.6f,theta_rate_rad_s=%.6f,ble_connected=%d,ble_subscribed=%d,fault_mask=0x%08lx,fault_first=%s\r\n",
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
      static_cast<unsigned long>(encoder_read_errors), imu_sample_valid ? 1 : 0,
      attitude_state.valid ? 1 : 0, attitude_state.angle_rad,
      attitude_state.rate_rad_s, triwhirl::ble::connected() ? 1 : 0,
      triwhirl::ble::subscribed() ? 1 : 0,
      static_cast<unsigned long>(safety_latch.mask()),
      triwhirl::safetyFaultName(safety_latch.firstFault()));
}

void printImuStatus() {
  std::uint8_t who_am_i = 0U;
  const bool who_ok = imu_ready && imu.readWhoAmI(&who_am_i);
  consolePrintf(
      "imu,ready=%d,sample_ok=%d,who_ok=%d,who=0x%02x,bias_valid=%d,calibrating=%d,ax=%.6f,ay=%.6f,az=%.6f,gx=%.6f,gy=%.6f,gz=%.6f,temp_c=%.3f,bx=%.6f,by=%.6f,bz=%.6f,map=%d:%d:%d:%d:%d:%d,read_errors=%lu\r\n",
      imu_ready ? 1 : 0, imu_sample_valid ? 1 : 0, who_ok ? 1 : 0,
      static_cast<unsigned>(who_am_i), gyro_bias_valid ? 1 : 0,
      gyro_calibration.active ? 1 : 0, imu_sample.accel_mps2[0],
      imu_sample.accel_mps2[1], imu_sample.accel_mps2[2], correctedGyro(0),
      correctedGyro(1), correctedGyro(2), imu_sample.temperature_c,
      gyro_bias_rad_s[0], gyro_bias_rad_s[1], gyro_bias_rad_s[2],
      imu_map.accel_sin_axis, imu_map.accel_cos_axis, imu_map.gyro_axis,
      imu_map.accel_sin_sign, imu_map.accel_cos_sign, imu_map.gyro_sign,
      static_cast<unsigned long>(imu_read_errors));
}

void printAttitudeStatus() {
  consolePrintf(
      "attitude,initialized=%d,valid=%d,theta_rad=%.6f,rate_rad_s=%.6f,residual_bias_rad_s=%.6f,innovation=%.6f,accel_weight=%.6f,wheel_rate_rad_s=%.6f\r\n",
      attitude_initialized ? 1 : 0, attitude_state.valid ? 1 : 0,
      attitude_state.angle_rad, attitude_state.rate_rad_s,
      attitude_state.gyro_bias_rad_s, attitude_state.gravity_innovation,
      attitude_state.accel_weight, wheel_state.velocity_rad_s);
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
    const float requested_vq =
        clampFinite(std::strtof(vq_token, nullptr), -kMotorVectorLimitV,
                    kMotorVectorLimitV);
    if (std::fabs(requested_vq) < 1.0e-4F) {
      stopMotor();
      consoleWrite("OK motor stop\r\n");
      return;
    }
    if (!motorStartAllowed()) {
      return;
    }
    if (!motor_config_valid) {
      consoleWrite("ERR motor is not calibrated/configured\r\n");
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
    if (!motorStartAllowed()) {
      return;
    }
    stopMotor();
    startCalibration(amplitude_token, hz_token, turns_token);
    return;
  }

  consoleWrite("ERR usage: motor <calibrate|config|vq|status|stop>\r\n");
}

void handleImuCommand() {
  char* action = std::strtok(nullptr, " \t");
  if (action == nullptr || std::strcmp(action, "status") == 0) {
    printImuStatus();
    return;
  }

  if (std::strcmp(action, "calibrate") == 0) {
    char* samples_token = std::strtok(nullptr, " \t");
    std::uint32_t samples = kDefaultGyroCalibrationSamples;
    if (samples_token != nullptr) {
      samples = static_cast<std::uint32_t>(std::strtoul(samples_token, nullptr, 10));
    }
    startGyroCalibration(samples);
    return;
  }

  if (std::strcmp(action, "map") == 0) {
    char* sin_axis_token = std::strtok(nullptr, " \t");
    char* cos_axis_token = std::strtok(nullptr, " \t");
    char* gyro_axis_token = std::strtok(nullptr, " \t");
    char* sin_sign_token = std::strtok(nullptr, " \t");
    char* cos_sign_token = std::strtok(nullptr, " \t");
    char* gyro_sign_token = std::strtok(nullptr, " \t");
    if (sin_axis_token == nullptr || cos_axis_token == nullptr ||
        gyro_axis_token == nullptr || sin_sign_token == nullptr ||
        cos_sign_token == nullptr || gyro_sign_token == nullptr) {
      consoleWrite("ERR usage: imu map <sin_axis> <cos_axis> <gyro_axis> <sin_sign> <cos_sign> <gyro_sign>\r\n");
      return;
    }
    ImuPlanarMap map{};
    map.accel_sin_axis = std::atoi(sin_axis_token);
    map.accel_cos_axis = std::atoi(cos_axis_token);
    map.gyro_axis = std::atoi(gyro_axis_token);
    map.accel_sin_sign = std::atoi(sin_sign_token);
    map.accel_cos_sign = std::atoi(cos_sign_token);
    map.gyro_sign = std::atoi(gyro_sign_token);
    if (!validImuMap(map)) {
      consoleWrite("ERR invalid imu map\r\n");
      return;
    }
    imu_map = map;
    resetAttitudeFromAccel();
    consolePrintf("OK imu map %d %d %d %d %d %d\r\n",
                  imu_map.accel_sin_axis, imu_map.accel_cos_axis,
                  imu_map.gyro_axis, imu_map.accel_sin_sign,
                  imu_map.accel_cos_sign, imu_map.gyro_sign);
    return;
  }

  consoleWrite("ERR usage: imu <status|calibrate [samples]|map ...>\r\n");
}

void handleAttitudeCommand() {
  char* action = std::strtok(nullptr, " \t");
  if (action == nullptr || std::strcmp(action, "status") == 0) {
    printAttitudeStatus();
    return;
  }

  if (std::strcmp(action, "reset") == 0) {
    char* angle_token = std::strtok(nullptr, " \t");
    if (angle_token == nullptr) {
      resetAttitudeFromAccel();
      consoleWrite("OK attitude reset from accelerometer\r\n");
      return;
    }
    const float angle = std::strtof(angle_token, nullptr);
    if (!std::isfinite(angle)) {
      consoleWrite("ERR invalid attitude angle\r\n");
      return;
    }
    attitude_estimator.reset(angle, 0.0F);
    attitude_state = attitude_estimator.state();
    attitude_initialized = true;
    last_attitude_update_us = static_cast<std::uint32_t>(esp_timer_get_time());
    consolePrintf("OK attitude reset angle_rad=%.6f\r\n", angle);
    return;
  }

  consoleWrite("ERR usage: attitude <status|reset [angle_rad]>\r\n");
}

void handleTimingCommand() {
  char* action = std::strtok(nullptr, " \t");
  if (action == nullptr || std::strcmp(action, "status") == 0) {
    printTimingStatus();
    return;
  }
  if (std::strcmp(action, "reset") == 0) {
    resetTimingStats();
    consoleWrite("OK timing reset\r\n");
    return;
  }
  consoleWrite("ERR usage: timing <status|reset>\r\n");
}

void handleFaultCommand() {
  char* action = std::strtok(nullptr, " \t");
  if (action == nullptr || std::strcmp(action, "status") == 0) {
    printFaultStatus();
    return;
  }
  if (std::strcmp(action, "clear") == 0) {
    stopMotor();
    if (!safety_latch.faulted()) {
      consoleWrite("OK fault already clear\r\n");
      return;
    }
    if (!faultClearReady()) {
      consoleWrite("ERR fault clear rejected; fault cause is still present\r\n");
      return;
    }
    safety_latch.clear();
    consoleWrite("OK fault clear\r\n");
    return;
  }
  consoleWrite("ERR usage: fault <status|clear>\r\n");
}

void handleBleCommand() {
  char* action = std::strtok(nullptr, " \t");
  if (action == nullptr || std::strcmp(action, "status") == 0) {
    printBleStatus();
    return;
  }
  consoleWrite("ERR usage: ble status\r\n");
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

  if (std::strcmp(command, "imu") == 0) {
    handleImuCommand();
    return;
  }

  if (std::strcmp(command, "attitude") == 0) {
    handleAttitudeCommand();
    return;
  }

  if (std::strcmp(command, "timing") == 0) {
    handleTimingCommand();
    return;
  }

  if (std::strcmp(command, "fault") == 0) {
    handleFaultCommand();
    return;
  }

  if (std::strcmp(command, "ble") == 0) {
    handleBleCommand();
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
    if (!motorStartAllowed()) {
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

void consumeConsoleBytes(const std::uint8_t* input,
                         const std::size_t received,
                         CommandInputState& state) {
  if (input == nullptr) {
    return;
  }

  for (std::size_t i = 0; i < received; ++i) {
    const char c = static_cast<char>(input[i]);

    if (c == '\r' || c == '\n') {
      if (state.length > 0U) {
        consoleWrite("\r\n");
        state.line[state.length] = '\0';
        handleCommand(state.line);
        state.length = 0U;
        printPrompt();
      }
      continue;
    }

    if (c == '\b' || static_cast<unsigned char>(c) == 0x7FU) {
      if (state.length > 0U) {
        --state.length;
        consoleWrite("\b \b");
      }
      continue;
    }

    if (c < 0x20 || static_cast<unsigned char>(c) > 0x7EU) {
      continue;
    }

    if (state.length + 1U < sizeof(state.line)) {
      state.line[state.length++] = c;
      consoleWriteBytes(&c, 1U);
    } else {
      state.length = 0U;
      consoleWrite("\r\nERR command too long\r\n");
      printPrompt();
    }
  }
}

void pollConsole() {
  std::uint8_t input[64];
  const int uart_received = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
  if (uart_received > 0) {
    consumeConsoleBytes(input, static_cast<std::size_t>(uart_received),
                        uart_command_input);
  }

  const std::size_t ble_received = triwhirl::ble::read(input, sizeof(input));
  if (ble_received > 0U) {
    consumeConsoleBytes(input, ble_received, ble_command_input);
  }
}

void emitTelemetry(const std::uint32_t now_us) {
  if (!telemetry_enabled ||
      (now_us - last_telemetry_us) < kTelemetryPeriodUs) {
    return;
  }
  last_telemetry_us = now_us;
  consolePrintf(
      "telemetry,%lu,%s,%.6f,%.6f,%.6f,%d,%d,%d,%u,%lld,%.6f,%.6f,%.6f,%.6f,%d,%lu,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%lu,%d,%.6f,%.6f,%.6f,%lu,%lu,%llu,%lu\r\n",
      static_cast<unsigned long>(now_us), motorModeName(motor_mode),
      vq_command_v, electrical_angle_rad, open_loop_hz,
      encoder_status_valid ? 1 : 0, encoder_sample_valid ? 1 : 0,
      encoder_status.magnet_detected ? 1 : 0,
      static_cast<unsigned>(wheel_state.raw_count),
      static_cast<long long>(wheel_state.unwrapped_count), wheel_state.angle_rad,
      wheel_state.unwrapped_angle_rad, wheel_state.velocity_rad_s,
      wheel_state.instantaneous_velocity_rad_s,
      wheel_state.velocity_valid ? 1 : 0,
      static_cast<unsigned long>(encoder_read_errors), imu_sample_valid ? 1 : 0,
      imu_sample.accel_mps2[0], imu_sample.accel_mps2[1], imu_sample.accel_mps2[2],
      correctedGyro(0), correctedGyro(1), correctedGyro(2),
      static_cast<unsigned long>(imu_read_errors), attitude_state.valid ? 1 : 0,
      attitude_state.angle_rad, attitude_state.rate_rad_s,
      attitude_state.accel_weight,
      static_cast<unsigned long>(timing_stats.last_exec_us),
      static_cast<unsigned long>(timing_stats.max_exec_us),
      static_cast<unsigned long long>(timing_stats.overruns),
      static_cast<unsigned long>(safety_latch.mask()));
}

void updateTimingStats(const std::int64_t start_us, const std::int64_t end_us) {
  const std::uint32_t exec_us = end_us > start_us
                                    ? static_cast<std::uint32_t>(end_us - start_us)
                                    : 0U;
  timing_stats.last_exec_us = exec_us;
  if (exec_us > timing_stats.max_exec_us) {
    timing_stats.max_exec_us = exec_us;
  }
  if (exec_us > kControlPeriodUs) {
    ++timing_stats.overruns;
  }

  if (timing_stats.previous_start_us != 0 && start_us > timing_stats.previous_start_us) {
    const std::uint32_t period_us =
        static_cast<std::uint32_t>(start_us - timing_stats.previous_start_us);
    if (period_us < timing_stats.min_period_us) {
      timing_stats.min_period_us = period_us;
    }
    if (period_us > timing_stats.max_period_us) {
      timing_stats.max_period_us = period_us;
    }
    if (period_us > kControlPeriodUs + 250U) {
      ++timing_stats.late_periods;
    }
  }
  timing_stats.previous_start_us = start_us;
  ++timing_stats.iterations;
}

void controlTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    const std::int64_t start_us = esp_timer_get_time();
    const std::uint32_t loop_us = static_cast<std::uint32_t>(start_us);
    updateEncoder(loop_us);
    updateImu(loop_us);
    evaluateSafety(start_us);
    updateMotor(loop_us);
    pollConsole();
    emitTelemetry(loop_us);
    const std::int64_t end_us = esp_timer_get_time();
    updateTimingStats(start_us, end_us);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
  }
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

bool initImuBus(i2c_master_bus_handle_t* bus) {
  i2c_master_bus_config_t config{};
  config.i2c_port = I2C_NUM_1;
  config.sda_io_num = static_cast<gpio_num_t>(triwhirl::board::kMpu6050SdaGpio);
  config.scl_io_num = static_cast<gpio_num_t>(triwhirl::board::kMpu6050SclGpio);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;
  return i2c_new_master_bus(&config, bus) == ESP_OK;
}

}  // namespace

extern "C" void app_main(void) {
  if (!initConsole()) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }

  console_tx_stream = xStreamBufferCreate(kConsoleTxBufferBytes, 1U);
  if (console_tx_stream == nullptr) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }
  if (xTaskCreatePinnedToCore(consoleTxTask, "triwhirl_uart_tx", 4096, nullptr,
                              2, nullptr, 0) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }

  if (!triwhirl::ble::init()) {
    consoleWrite("WARN BLE init failed; Web Bluetooth unavailable\r\n");
  }

  i2c_master_bus_handle_t encoder_bus = nullptr;
  if (!initEncoderBus(&encoder_bus) ||
      !encoder.init(encoder_bus, triwhirl::board::kAs5600I2cAddress)) {
    safety_latch.trip(SafetyFault::kStartup);
    consoleWrite("FATAL fault=startup AS5600 I2C init failed\r\n");
    return;
  }

  i2c_master_bus_handle_t imu_bus = nullptr;
  imu_ready = initImuBus(&imu_bus) &&
              imu.init(imu_bus, triwhirl::board::kMpu6050I2cAddress);
  if (!imu_ready) {
    consoleWrite("WARN MPU6050 init failed; IMU functions unavailable\r\n");
  }

  if (!bridge.init(triwhirl::board::kMotorIn1Gpio,
                   triwhirl::board::kMotorIn2Gpio,
                   triwhirl::board::kMotorIn3Gpio, kPwmFrequencyHz,
                   triwhirl::board::kMotorBusNominalV)) {
    safety_latch.trip(SafetyFault::kActuator);
    consoleWrite("FATAL fault=actuator MCPWM bridge init failed\r\n");
    return;
  }

  bridge.stopZeroVector();
  const std::uint32_t now_us = static_cast<std::uint32_t>(esp_timer_get_time());
  sampleEncoder(now_us);
  refreshEncoderHealth();
  if (imu_ready) {
    sampleImu();
    startGyroCalibration(kDefaultGyroCalibrationSamples);
  }
  last_motor_update_us = now_us;
  last_encoder_sample_us = now_us;
  last_encoder_health_us = now_us;
  last_imu_sample_us = now_us;
  last_telemetry_us = now_us;

  consoleWrite("TriWhirl deterministic motor + IMU + attitude runtime ready\r\n");
  consoleWrite("UART + Web Bluetooth share the same command/telemetry protocol\r\n");
  consoleWrite("motor actuation is inhibited while a safety fault is latched\r\n");
  consoleWrite("telemetry is off by default; use 'telemetry on' when streaming is needed\r\n");
  consoleWrite("telemetry_fields,t_us,mode,vq_v,e_angle_rad,e_hz,status_ok,sample_ok,mag,raw,unwrapped_count,angle_rad,unwrapped_rad,vel_rad_s,vel_inst_rad_s,vel_valid,read_errors,imu_ok,ax,ay,az,gx,gy,gz,imu_read_errors,attitude_ok,theta_rad,theta_rate_rad_s,accel_weight,loop_exec_us,loop_max_exec_us,loop_overruns,fault_mask\r\n");
  printStatus();
  printHelp();
  printPrompt();

  if (xTaskCreatePinnedToCore(controlTask, "triwhirl_control", 8192, nullptr,
                              configMAX_PRIORITIES - 2, nullptr, 1) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup control task creation failed\r\n");
    return;
  }

  vTaskDelete(nullptr);
}
