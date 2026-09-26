#include "runtime_balance.hpp"

#include <cmath>

#include "esp_timer.h"
#include "runtime_egress.hpp"
#include "runtime_release.hpp"
#include "runtime_sensor_frame.hpp"
#include "runtime_sensor_pipeline.hpp"
#include "runtime_standup.hpp"
#include "runtime_state.hpp"
#include "triwhirl/safety.hpp"

namespace triwhirl::runtime {
namespace {

constexpr std::uint32_t kBalanceSensorFreshnessUs = 3000U;
constexpr float kVendorStandupWheelLimitRadS = 160.0F;

triwhirl::BalanceControllerConfig balance_config{};
bool balance_config_valid = false;
bool balance_active = false;
bool vendor_standup_mode = false;
triwhirl::BalanceControllerOutput balance_output{};

triwhirl::BalanceControllerInput currentBalanceInput() {
  triwhirl::BalanceControllerInput input{};
  input.theta_rad = state::attitude_state.angle_rad;
  input.theta_rate_rad_s = state::attitude_state.rate_rad_s;
  input.wheel_rate_rad_s = state::wheel_state.velocity_rad_s;
  return input;
}

bool consumedBalanceSensorFrame(RuntimeSensorFrame* const frame) {
  RuntimeSensorFrame consumed{};
  if (!readLastConsumedSensorFrame(&consumed) || !consumed.complete ||
      !consumed.imu_expected) {
    if (frame != nullptr) *frame = consumed;
    return false;
  }

  // Synthesized direct-Vq Balance validates the exact generation consumed by
  // Core 1 against the original 3 ms nominal age budget. The vendor standup
  // fallback deliberately uses the generic bounded freshness path because the
  // seller controller is proven on a slower, non-deterministic Arduino loop.
  const std::uint32_t now_us =
      static_cast<std::uint32_t>(esp_timer_get_time());
  const bool encoder_fresh = sensorTimestampNominallyFresh(
      now_us, encoderSampleTimestampUs(consumed.encoder),
      kBalanceSensorFreshnessUs);
  const bool imu_fresh = sensorTimestampNominallyFresh(
      now_us, imuSampleTimestampUs(consumed.imu), kBalanceSensorFreshnessUs);

  if (frame != nullptr) {
    *frame = consumed;
    if (!encoder_fresh) frame->encoder.ok = false;
    if (!imu_fresh) frame->imu.ok = false;
  }
  return encoder_fresh && imu_fresh;
}

void tripBalanceFault(const triwhirl::SafetyFault fault) {
  const std::uint32_t before = state::safety_latch.mask();
  state::safety_latch.trip(fault);
  balance_active = false;
  vendor_standup_mode = false;
  stopRuntimeStandup();
  state::stopMotor();
  if ((before & triwhirl::safetyFaultMask(fault)) == 0U) {
    RuntimeStateEvent event{};
    event.type = RuntimeStateEventType::kFaultLatched;
    event.value0 = static_cast<std::int32_t>(fault);
    event.u32_0 = state::safety_latch.mask();
    publishRuntimeStateEvent(event);
  }
}

void tripIncompleteBalanceSensorFrame(const RuntimeSensorFrame& frame) {
  if (!frame.encoder_received || !frame.encoder.ok) {
    tripBalanceFault(triwhirl::SafetyFault::kEncoderUnavailable);
    return;
  }
  tripBalanceFault(triwhirl::SafetyFault::kImuUnavailable);
}

BalanceStartFailure mapStandupStartFailure(const StandupStartFailure failure) {
  switch (failure) {
    case StandupStartFailure::kNone: return BalanceStartFailure::kNone;
    case StandupStartFailure::kAlreadyActive:
      return BalanceStartFailure::kAlreadyActive;
    case StandupStartFailure::kMotorActive:
      return BalanceStartFailure::kMotorActive;
    case StandupStartFailure::kReleaseClock:
      return BalanceStartFailure::kReleaseClock;
    case StandupStartFailure::kMotorConfig:
      return BalanceStartFailure::kMotorConfig;
    case StandupStartFailure::kEncoder: return BalanceStartFailure::kEncoder;
    case StandupStartFailure::kImu: return BalanceStartFailure::kImu;
    case StandupStartFailure::kAttitude: return BalanceStartFailure::kAttitude;
    case StandupStartFailure::kSafetyFault:
      return BalanceStartFailure::kSafetyFault;
  }
  return BalanceStartFailure::kControllerConfig;
}

void mirrorStandupOutput() {
  const auto status = runtimeStandupStatus();
  const float abs_error = std::fabs(status.output.theta_error_rad);
  const float abs_wheel_rate = std::fabs(state::wheel_state.velocity_rad_s);

  balance_output = {};
  balance_output.valid = status.output.valid;
  balance_output.capture_ready =
      status.output.phase == triwhirl::StandupPhase::kBalance &&
      abs_error <= status.config.balance_capture_rad &&
      abs_wheel_rate <= kVendorStandupWheelLimitRadS;
  balance_output.inside_envelope =
      abs_error <= status.config.settling_fall_rad &&
      abs_wheel_rate <= kVendorStandupWheelLimitRadS;
  balance_output.theta_error_rad = status.output.theta_error_rad;
  balance_output.vq_unsaturated_v = status.output.vq_unclamped_v;
  balance_output.vq_v = status.output.vq_v;
}

}  // namespace

bool configureRuntimeBalance(const triwhirl::BalanceControllerConfig& config) {
  if (balance_active || runtimeStandupActive() || state::motorActive() ||
      !triwhirl::validBalanceControllerConfig(config) ||
      config.vq_limit_v > state::kMotorVectorLimitV) {
    return false;
  }
  balance_config = config;
  balance_config_valid = true;
  vendor_standup_mode = false;
  balance_output = {};
  return true;
}

BalanceStartFailure startRuntimeBalance(float* const initial_vq_v) {
  if (balance_active) return BalanceStartFailure::kAlreadyActive;
  if (state::motorActive()) return BalanceStartFailure::kMotorActive;
  if (!realtimeReleaseReady()) return BalanceStartFailure::kReleaseClock;

  // No synthesized controller has been configured after boot: start the known-
  // good seller architecture instead. This keeps identification/H-infinity
  // deployment untouched while enabling an immediate autonomous
  // swing-up -> local-balance trial on the proven TRC-V1.1 electrical/control
  // baseline.
  if (!balance_config_valid) {
    const StandupStartFailure standup_failure = startRuntimeStandup();
    const BalanceStartFailure failure = mapStandupStartFailure(standup_failure);
    if (failure != BalanceStartFailure::kNone) return failure;
    vendor_standup_mode = true;
    balance_active = true;
    mirrorStandupOutput();
    if (initial_vq_v != nullptr) *initial_vq_v = state::vq_command_v;
    return BalanceStartFailure::kNone;
  }

  if (!triwhirl::validBalanceControllerConfig(balance_config) ||
      balance_config.vq_limit_v > state::kMotorVectorLimitV) {
    return BalanceStartFailure::kControllerConfig;
  }
  if (!state::motor_config_valid) return BalanceStartFailure::kMotorConfig;
  if (!state::encoder_sample_valid || !state::wheel_state.velocity_valid) {
    return BalanceStartFailure::kEncoder;
  }
  if (!state::imu_ready || !state::imu_sample_valid || !state::gyro_bias_valid) {
    return BalanceStartFailure::kImu;
  }
  if (!state::attitude_initialized || !state::attitude_state.valid) {
    return BalanceStartFailure::kAttitude;
  }
  if (state::safety_latch.faulted()) return BalanceStartFailure::kSafetyFault;
  if (!consumedBalanceSensorFrame(nullptr)) return BalanceStartFailure::kSensorFrame;

  const triwhirl::BalanceControllerInput input = currentBalanceInput();
  const triwhirl::BalanceControllerOutput output =
      triwhirl::evaluateBalanceController(balance_config, input);
  if (!output.valid) return BalanceStartFailure::kControllerConfig;
  if (std::fabs(output.theta_error_rad) > balance_config.capture_angle_rad) {
    return BalanceStartFailure::kOutsideCapture;
  }
  if (std::fabs(input.wheel_rate_rad_s) >
      balance_config.wheel_rate_limit_rad_s) {
    return BalanceStartFailure::kWheelRate;
  }

  balance_output = output;
  vendor_standup_mode = false;
  balance_active = true;
  state::vq_command_v = output.vq_v;
  state::motor_mode = state::MotorMode::kFoc;
  if (initial_vq_v != nullptr) {
    *initial_vq_v = output.vq_v;
  }
  return BalanceStartFailure::kNone;
}

void stopRuntimeBalance() {
  if (!balance_active && !runtimeStandupActive()) return;
  if (vendor_standup_mode || runtimeStandupActive()) {
    stopRuntimeStandup();
  }
  vendor_standup_mode = false;
  balance_active = false;
  balance_output = {};
  state::stopMotor();
}

void updateRuntimeBalance() {
  if (!balance_active) return;

  if (vendor_standup_mode) {
    updateRuntimeStandup(static_cast<std::uint32_t>(esp_timer_get_time()));
    if (!runtimeStandupActive()) {
      balance_active = false;
      vendor_standup_mode = false;
      balance_output = {};
      return;
    }
    mirrorStandupOutput();
    return;
  }

  if (!realtimeReleaseReady()) {
    tripBalanceFault(triwhirl::SafetyFault::kControlTiming);
    return;
  }
  if (state::safety_latch.faulted()) {
    balance_active = false;
    state::stopMotor();
    return;
  }
  if (state::motor_mode != state::MotorMode::kFoc) {
    balance_active = false;
    balance_output = {};
    return;
  }
  if (!state::encoder_sample_valid || !state::wheel_state.velocity_valid) {
    tripBalanceFault(triwhirl::SafetyFault::kEncoderUnavailable);
    return;
  }
  if (!state::imu_ready || !state::imu_sample_valid || !state::gyro_bias_valid) {
    tripBalanceFault(triwhirl::SafetyFault::kImuUnavailable);
    return;
  }
  if (!state::attitude_initialized || !state::attitude_state.valid) {
    tripBalanceFault(triwhirl::SafetyFault::kImuUnavailable);
    return;
  }

  RuntimeSensorFrame sensor_frame{};
  if (!consumedBalanceSensorFrame(&sensor_frame)) {
    tripIncompleteBalanceSensorFrame(sensor_frame);
    return;
  }

  const triwhirl::BalanceControllerInput input = currentBalanceInput();
  const triwhirl::BalanceControllerOutput output =
      triwhirl::evaluateBalanceController(balance_config, input);
  if (!output.valid || !std::isfinite(output.vq_v)) {
    tripBalanceFault(triwhirl::SafetyFault::kInvalidNumeric);
    return;
  }
  if (std::fabs(output.theta_error_rad) > balance_config.fall_angle_rad) {
    tripBalanceFault(triwhirl::SafetyFault::kBodyAngle);
    return;
  }
  if (std::fabs(input.wheel_rate_rad_s) >
      balance_config.wheel_rate_limit_rad_s) {
    tripBalanceFault(triwhirl::SafetyFault::kWheelOverspeed);
    return;
  }

  balance_output = output;
  state::vq_command_v = output.vq_v;
}

bool runtimeBalanceActive() { return balance_active; }

RuntimeBalanceStatus runtimeBalanceStatus() {
  RuntimeBalanceStatus status{};
  status.active = balance_active;
  if (vendor_standup_mode || runtimeStandupActive()) {
    const auto standup = runtimeStandupStatus();
    status.configured = true;
    status.config = {};
    status.config.theta_reference_rad = standup.output.theta_reference_rad;
    status.config.capture_angle_rad = standup.config.balance_capture_rad;
    status.config.fall_angle_rad = standup.config.settling_fall_rad;
    status.config.vq_limit_v = standup.config.vq_limit_v;
    status.config.wheel_rate_limit_rad_s = kVendorStandupWheelLimitRadS;
    status.output = balance_output;
    return status;
  }
  status.configured = balance_config_valid;
  status.config = balance_config;
  status.output = balance_output;
  return status;
}

const char* balanceStartFailureName(const BalanceStartFailure failure) {
  switch (failure) {
    case BalanceStartFailure::kNone: return "none";
    case BalanceStartFailure::kAlreadyActive: return "already_active";
    case BalanceStartFailure::kMotorActive: return "motor_active";
    case BalanceStartFailure::kReleaseClock: return "release_clock";
    case BalanceStartFailure::kControllerConfig: return "controller_config";
    case BalanceStartFailure::kMotorConfig: return "motor_config";
    case BalanceStartFailure::kEncoder: return "encoder";
    case BalanceStartFailure::kImu: return "imu";
    case BalanceStartFailure::kAttitude: return "attitude";
    case BalanceStartFailure::kSafetyFault: return "safety_fault";
    case BalanceStartFailure::kSensorFrame: return "sensor_frame";
    case BalanceStartFailure::kOutsideCapture: return "outside_capture";
    case BalanceStartFailure::kWheelRate: return "wheel_rate";
  }
  return "unknown";
}

}  // namespace triwhirl::runtime