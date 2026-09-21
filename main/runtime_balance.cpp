#include "runtime_balance.hpp"

#include <cmath>

#include "runtime_egress.hpp"
#include "runtime_release.hpp"
#include "runtime_state.hpp"
#include "triwhirl/safety.hpp"

namespace triwhirl::runtime {
namespace {

triwhirl::BalanceControllerConfig balance_config{};
bool balance_config_valid = false;
bool balance_active = false;
triwhirl::BalanceControllerOutput balance_output{};

triwhirl::BalanceControllerInput currentBalanceInput() {
  triwhirl::BalanceControllerInput input{};
  input.theta_rad = state::attitude_state.angle_rad;
  input.theta_rate_rad_s = state::attitude_state.rate_rad_s;
  input.wheel_rate_rad_s = state::wheel_state.velocity_rad_s;
  return input;
}

void tripBalanceFault(const triwhirl::SafetyFault fault) {
  const std::uint32_t before = state::safety_latch.mask();
  state::safety_latch.trip(fault);
  balance_active = false;
  state::stopMotor();
  if ((before & triwhirl::safetyFaultMask(fault)) == 0U) {
    RuntimeStateEvent event{};
    event.type = RuntimeStateEventType::kFaultLatched;
    event.value0 = static_cast<std::int32_t>(fault);
    event.u32_0 = state::safety_latch.mask();
    publishRuntimeStateEvent(event);
  }
}

}  // namespace

bool configureRuntimeBalance(const triwhirl::BalanceControllerConfig& config) {
  if (balance_active || state::motorActive() ||
      !triwhirl::validBalanceControllerConfig(config) ||
      config.vq_limit_v > state::kMotorVectorLimitV) {
    return false;
  }
  balance_config = config;
  balance_config_valid = true;
  balance_output = {};
  return true;
}

BalanceStartFailure startRuntimeBalance(float* const initial_vq_v) {
  if (balance_active) return BalanceStartFailure::kAlreadyActive;
  if (state::motorActive()) return BalanceStartFailure::kMotorActive;
  if (!realtimeReleaseReady()) return BalanceStartFailure::kReleaseClock;
  if (!balance_config_valid ||
      !triwhirl::validBalanceControllerConfig(balance_config) ||
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
  balance_active = true;
  state::vq_command_v = output.vq_v;
  state::motor_mode = state::MotorMode::kFoc;
  if (initial_vq_v != nullptr) {
    *initial_vq_v = output.vq_v;
  }
  return BalanceStartFailure::kNone;
}

void stopRuntimeBalance() {
  if (!balance_active) return;
  balance_active = false;
  balance_output = {};
  state::stopMotor();
}

void updateRuntimeBalance() {
  if (!balance_active) return;

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
  status.configured = balance_config_valid;
  status.active = balance_active;
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
    case BalanceStartFailure::kOutsideCapture: return "outside_capture";
    case BalanceStartFailure::kWheelRate: return "wheel_rate";
  }
  return "unknown";
}

}  // namespace triwhirl::runtime
