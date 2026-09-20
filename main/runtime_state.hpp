#pragma once

#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "triwhirl/attitude_estimator.hpp"
#include "triwhirl/drivers/as5600.hpp"
#include "triwhirl/drivers/mpu6050.hpp"
#include "triwhirl/motor/three_pwm_bridge.hpp"
#include "triwhirl/runtime_logger.hpp"
#include "triwhirl/safety.hpp"
#include "triwhirl/voltage_mode_foc.hpp"
#include "triwhirl/wheel_kinematics.hpp"

namespace triwhirl::runtime::state {

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
using triwhirl::log::LoggerStatus;
using triwhirl::log::RuntimeLogRecord;
using triwhirl::log::RuntimeLogger;
using triwhirl::motor::ThreePwmBridge;

inline constexpr float kTwoPi = 6.28318530717958647692F;
inline constexpr float kMaxElectricalHz = 30.0F;
inline constexpr float kWheelVelocityFilterTauS = 0.01F;
inline constexpr float kMotorVectorLimitV = triwhirl::board::kBringupPhaseAmplitudeMaxV;
inline constexpr float kDefaultCalibrationAmplitudeV = 0.6F;
inline constexpr float kDefaultCalibrationElectricalHz = 0.5F;
inline constexpr float kDefaultCalibrationTurns = 4.0F;
inline constexpr std::uint32_t kCalibrationAlignUs = 500000U;
inline constexpr std::uint32_t kCalibrationSettleUs = 400000U;
inline constexpr std::uint32_t kControlPeriodUs = 1000U;
inline constexpr std::uint32_t kHardControlPeriodUs = 20U * kControlPeriodUs;
inline constexpr std::uint32_t kDefaultGyroCalibrationSamples = 500U;
inline constexpr std::uint32_t kTelemetryPeriodUs = 20000U;
inline constexpr std::uint32_t kPwmFrequencyHz = 25000U;
inline constexpr std::size_t kConsoleTxBufferBytes = 8192U;

// State formerly hidden inside the legacy app_main.cpp translation-unit bridge.
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
  std::uint32_t min_period_us = UINT32_MAX;
  std::uint32_t max_period_us = 0U;
  std::int64_t previous_start_us = 0;
};

extern As5600 encoder;
extern Mpu6050 imu;
extern WheelKinematics wheel_kinematics;
extern PlanarAttitudeEstimator attitude_estimator;
extern ThreePwmBridge bridge;
extern SafetyLatch safety_latch;
extern RuntimeLogger runtime_logger;

extern As5600Status encoder_status;
extern WheelKinematicsState wheel_state;
extern bool encoder_status_valid;
extern bool encoder_sample_valid;
extern std::uint32_t encoder_read_errors;

extern Mpu6050Sample imu_sample;
extern bool imu_ready;
extern bool imu_sample_valid;
extern bool gyro_bias_valid;
extern float gyro_bias_rad_s[3];
extern std::uint32_t imu_read_errors;
extern GyroCalibrationState gyro_calibration;
extern ImuPlanarMap imu_map;
extern AttitudeEstimate attitude_state;
extern bool attitude_initialized;
extern std::uint32_t last_attitude_update_us;

extern MotorMode motor_mode;
extern MotorElectricalConfig motor_config;
extern bool motor_config_valid;
extern CalibrationState calibration;

extern bool telemetry_enabled;
extern float open_loop_amplitude_v;
extern float open_loop_hz;
extern float open_loop_angle_rad;
extern float vq_command_v;
extern float electrical_angle_rad;
extern std::uint32_t last_motor_update_us;
extern std::uint32_t last_telemetry_us;

extern ControlTimingStats timing_stats;
extern StreamBufferHandle_t console_tx_stream;
extern std::uint32_t console_tx_dropped_bytes;
extern volatile bool log_critical_window;
extern volatile bool binary_dump_active;
extern TaskHandle_t log_dump_task;

void consoleWriteBytes(const char* data, std::size_t length);
void consoleWrite(const char* text);
void consolePrintf(const char* format, ...);
void consoleTxTask(void*);
const char* motorModeName(MotorMode mode);
void printPrompt();
float clampFinite(float value, float low, float high);
bool validImuMap(const ImuPlanarMap& map);
bool refreshEncoderHealth();
bool sampleEncoder(std::uint32_t sample_time_us);
float correctedGyro(int axis);
void resetAttitudeFromAccel();
void updateAttitude(std::uint32_t now_us);
void startGyroCalibration(std::uint32_t samples);
bool sampleImu();
void stopMotor();
bool motorActive();
bool motorStartAllowed();
bool faultClearReady();
void evaluateSafety(std::int64_t start_us);
void updateMotor(std::uint32_t now_us);
std::uint16_t runtimeLogFlags();
void printLogStatus();
void logDumpTask(void*);
void printStatus();
void printImuStatus();
void resetTimingStats();
void emitTelemetry(std::uint32_t now_us);
void updateTimingStats(std::int64_t start_us, std::int64_t end_us);
bool initConsole();
void printHelp();

}  // namespace triwhirl::runtime::state
