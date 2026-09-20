#include "runtime_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/board.hpp"

namespace triwhirl::runtime::state {
namespace {

bool validAxis(const int axis) { return axis >= 0 && axis <= 2; }
bool validSign(const int sign) { return sign == 1 || sign == -1; }

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
      !std::isfinite(open_loop_angle_rad) || !std::isfinite(vq_command_v) ||
      !std::isfinite(electrical_angle_rad)) {
    return false;
  }
  if (wheel_state.velocity_valid &&
      (!std::isfinite(wheel_state.velocity_rad_s) ||
       !std::isfinite(wheel_state.instantaneous_velocity_rad_s))) {
    return false;
  }
  return !attitude_state.valid ||
         (std::isfinite(attitude_state.angle_rad) &&
          std::isfinite(attitude_state.rate_rad_s));
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
  return static_cast<float>(imu_map.gyro_sign) * correctedGyro(imu_map.gyro_axis);
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
    applyDq(calibration.commanded_electrical_rad, calibration.amplitude_v, 0.0F);
    return;
  }

  if (calibration.stage == CalibrationStage::kSettle &&
      applyDq(calibration.commanded_electrical_rad, calibration.amplitude_v,
              0.0F) &&
      (now_us - calibration.stage_start_us) >= kCalibrationSettleUs) {
    finishCalibration();
  }
}

}  // namespace

As5600 encoder;
Mpu6050 imu;
WheelKinematics wheel_kinematics(kWheelVelocityFilterTauS);
PlanarAttitudeEstimator attitude_estimator;
ThreePwmBridge bridge;
SafetyLatch safety_latch;
RuntimeLogger runtime_logger;

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
std::uint32_t last_telemetry_us = 0U;

ControlTimingStats timing_stats{};
StreamBufferHandle_t console_tx_stream = nullptr;
std::uint32_t console_tx_dropped_bytes = 0U;
volatile bool log_critical_window = false;
volatile bool binary_dump_active = false;
TaskHandle_t log_dump_task = nullptr;

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
  if (!binary_dump_active) {
    triwhirl::ble::write(reinterpret_cast<const std::uint8_t*>(data), length);
  }
}

void consoleWrite(const char* text) {
  if (text != nullptr) {
    consoleWriteBytes(text, std::strlen(text));
  }
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
    case MotorMode::kStopped: return "stopped";
    case MotorMode::kOpenLoop: return "open_loop";
    case MotorMode::kFoc: return "foc";
    case MotorMode::kCalibrating: return "calibrating";
  }
  return "unknown";
}

void printPrompt() {
  if (!telemetry_enabled && !binary_dump_active) {
    consoleWrite("> ");
  }
}

float clampFinite(const float value, const float low, const float high) {
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  return value < low ? low : (value > high ? high : value);
}

bool validImuMap(const ImuPlanarMap& map) {
  return validAxis(map.accel_sin_axis) && validAxis(map.accel_cos_axis) &&
         validAxis(map.gyro_axis) && map.accel_sin_axis != map.accel_cos_axis &&
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
  attitude_state = attitude_estimator.update(
      mappedAccelSin(), mappedAccelCos(), mappedGyro(),
      static_cast<float>(elapsed_us) * 1.0e-6F, true);
}

void startGyroCalibration(std::uint32_t samples) {
  if (!imu_ready) {
    consoleWrite("ERR imu unavailable\r\n");
    return;
  }
  samples = std::max<std::uint32_t>(50U, std::min<std::uint32_t>(5000U, samples));
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

void stopMotor() {
  motor_mode = MotorMode::kStopped;
  calibration.stage = CalibrationStage::kIdle;
  open_loop_amplitude_v = 0.0F;
  open_loop_hz = 0.0F;
  vq_command_v = 0.0F;
  bridge.stopZeroVector();
}

bool motorActive() { return motor_mode != MotorMode::kStopped; }

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
  return !hasFault(SafetyFault::kImuUnavailable) || imu_sample_valid;
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
      start_us > timing_stats.previous_start_us &&
      static_cast<std::uint64_t>(start_us - timing_stats.previous_start_us) >
          kHardControlPeriodUs) {
    tripFault(SafetyFault::kControlTiming);
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

std::uint16_t runtimeLogFlags() {
  std::uint16_t flags = 0U;
  if (encoder_sample_valid) flags |= triwhirl::log::kRecordEncoderValid;
  if (wheel_state.velocity_valid) flags |= triwhirl::log::kRecordWheelRateValid;
  if (imu_sample_valid) flags |= triwhirl::log::kRecordImuValid;
  if (attitude_state.valid) flags |= triwhirl::log::kRecordAttitudeValid;
  if (motorActive()) flags |= triwhirl::log::kRecordMotorActive;
  if (motor_mode == MotorMode::kFoc) flags |= triwhirl::log::kRecordMotorFoc;
  else if (motor_mode == MotorMode::kOpenLoop) flags |= triwhirl::log::kRecordMotorOpenLoop;
  else if (motor_mode == MotorMode::kCalibrating) flags |= triwhirl::log::kRecordMotorCalibrating;
  if (safety_latch.faulted()) flags |= triwhirl::log::kRecordSafetyFaulted;
  if (log_critical_window) flags |= triwhirl::log::kRecordCriticalWindow;
  return flags;
}

void printLogStatus() {
  const LoggerStatus status = runtime_logger.status();
  consolePrintf(
      "log,state=%s,partition_bytes=%lu,prepared_bytes=%lu,max_records=%lu,buffered_bytes=%lu,records_written=%lu,dropped_records=%lu,logical_bytes=%lu,flash_write=%d,critical=%d,dump_active=%d\r\n",
      triwhirl::log::loggerStateName(status.state),
      static_cast<unsigned long>(status.partition_bytes),
      static_cast<unsigned long>(status.prepared_bytes),
      static_cast<unsigned long>(status.max_records),
      static_cast<unsigned long>(status.buffered_bytes),
      static_cast<unsigned long>(status.records_written),
      static_cast<unsigned long>(status.dropped_records),
      static_cast<unsigned long>(status.logical_bytes),
      status.flash_writes_allowed ? 1 : 0, log_critical_window ? 1 : 0,
      binary_dump_active ? 1 : 0);
}

void logDumpTask(void*) {
  const LoggerStatus status = runtime_logger.status();
  const std::uint32_t logical_size = runtime_logger.logicalSize();
  char marker[160];
  const int marker_length = std::snprintf(
      marker, sizeof(marker),
      "logdump,format=TWLG1,bytes=%lu,record_size=%u,records=%lu\r\n",
      static_cast<unsigned long>(logical_size),
      static_cast<unsigned>(triwhirl::log::kTwLogRecordBytes),
      static_cast<unsigned long>(status.records_written));
  bool ok = marker_length > 0 &&
      static_cast<std::size_t>(marker_length) < sizeof(marker) &&
      triwhirl::ble::writeBlocking(
          reinterpret_cast<const std::uint8_t*>(marker),
          static_cast<std::size_t>(marker_length), 5000U) ==
          static_cast<std::size_t>(marker_length);
  std::uint8_t buffer[512];
  std::uint32_t offset = 0U;
  while (ok && offset < logical_size) {
    const std::size_t count = std::min<std::size_t>(
        sizeof(buffer), static_cast<std::size_t>(logical_size - offset));
    if (!runtime_logger.readLogical(offset, buffer, count) ||
        triwhirl::ble::writeBlocking(buffer, count, 10000U) != count) {
      ok = false;
      break;
    }
    offset += static_cast<std::uint32_t>(count);
  }
  static constexpr char kEndMarker[] = "\r\nlogdump_end\r\n";
  if (ok) {
    triwhirl::ble::writeBlocking(
        reinterpret_cast<const std::uint8_t*>(kEndMarker),
        sizeof(kEndMarker) - 1U, 5000U);
  }
  binary_dump_active = false;
  log_dump_task = nullptr;
  if (!ok) {
    consoleWrite("ERR log dump aborted\r\n");
  }
  vTaskDelete(nullptr);
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

void resetTimingStats() {
  timing_stats = {};
  console_tx_dropped_bytes = 0U;
}

void emitTelemetry(const std::uint32_t now_us) {
  if (!telemetry_enabled || binary_dump_active ||
      (now_us - last_telemetry_us) < kTelemetryPeriodUs) {
    return;
  }
  last_telemetry_us = now_us;
  consolePrintf(
      "telemetry,%lu,%s,%.6f,%.6f,%.6f,%d,%d,%d,%u,%lld,%.6f,%.6f,%.6f,%.6f,%d,%lu,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%lu,%d,%.6f,%.6f,%.6f,%lu,%lu,%llu,%lu\r\n",
      static_cast<unsigned long>(now_us), motorModeName(motor_mode), vq_command_v,
      electrical_angle_rad, open_loop_hz, encoder_status_valid ? 1 : 0,
      encoder_sample_valid ? 1 : 0, encoder_status.magnet_detected ? 1 : 0,
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
  timing_stats.max_exec_us = std::max(timing_stats.max_exec_us, exec_us);
  if (exec_us > kControlPeriodUs) {
    ++timing_stats.overruns;
  }
  if (timing_stats.previous_start_us != 0 && start_us > timing_stats.previous_start_us) {
    const std::uint32_t period_us =
        static_cast<std::uint32_t>(start_us - timing_stats.previous_start_us);
    timing_stats.min_period_us = std::min(timing_stats.min_period_us, period_us);
    timing_stats.max_period_us = std::max(timing_stats.max_period_us, period_us);
    if (period_us > kControlPeriodUs + 250U) {
      ++timing_stats.late_periods;
    }
  }
  timing_stats.previous_start_us = start_us;
  ++timing_stats.iterations;
}

bool initConsole() {
  uart_config_t config{};
  config.baud_rate = 115200;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  if (uart_param_config(UART_NUM_0, &config) != ESP_OK ||
      uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
    return false;
  }
  const esp_err_t result = uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
  return result == ESP_OK || result == ESP_ERR_INVALID_STATE;
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
  consoleWrite("  log status\r\n");
  consoleWrite("  log prepare [seconds]\r\n");
  consoleWrite("  log start\r\n");
  consoleWrite("  log critical <on|off>\r\n");
  consoleWrite("  log stop\r\n");
  consoleWrite("  log dump\r\n");
  consoleWrite("  field <electrical_hz> <amplitude_v>\r\n");
  consoleWrite("  stop\r\n");
  consoleWrite("  status\r\n");
  consoleWrite("  telemetry [on|off]\r\n");
  consoleWrite("  help\r\n");
}

}  // namespace triwhirl::runtime::state
