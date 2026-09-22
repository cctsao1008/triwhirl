#include "runtime_standup.hpp"

#include <cmath>

#include "esp_timer.h"
#include "runtime_egress.hpp"
#include "runtime_release.hpp"
#include "runtime_state.hpp"
#include "triwhirl/safety.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace triwhirl::runtime {
namespace {

constexpr float kDefaultThetaReferenceRad = 68.0F * triwhirl::kPi / 180.0F;
// The vendor velocity target is limited to 140 rad/s. Keep a small physical
// margin above that target before treating wheel speed as a hard standup fault.
constexpr float kStandupWheelHardLimitRadS = 160.0F;

triwhirl::StandupControllerConfig standup_config{};
triwhirl::StandupController standup_controller{};
bool standup_configured = false;
bool standup_active = false;

triwhirl::StandupControllerInput currentStandupInput(
    const std::uint32_t now_us) {
  triwhirl::StandupControllerInput input{};
  input.now_us = now_us;
  input.theta_rad = state::attitude_state.angle_rad;
  input.theta_rate_rad_s = state::attitude_state.rate_rad_s;
  input.wheel_rate_rad_s = state::wheel_state.velocity_rad_s;
  input.valid = state::encoder_sample_valid && state::wheel_state.velocity_valid &&
                state::imu_ready && state::imu_sample_valid &&
                state::gyro_bias_valid && state::attitude_initialized &&
                state::attitude_state.valid && !state::safety_latch.faulted();
  return input;
}

void tripStandupFault(const triwhirl::SafetyFault fault) {
  const std::uint32_t before = state::safety_latch.mask();
  state::safety_latch.trip(fault);
  standup_active = false;
  state::stopMotor();
  if ((before & triwhirl::safetyFaultMask(fault)) == 0U) {
    RuntimeStateEvent event{};
    event.type = RuntimeStateEventType::kFaultLatched;
    event.value0 = static_cast<std::int32_t>(fault);
    event.u32_0 = state::safety_latch.mask();
    publishRuntimeStateEvent(event);
  }
}

void ensureDefaultConfig() {
  if (standup_configured) return;
  configureRuntimeStandup(kDefaultThetaReferenceRad);
}

}  // namespace

bool configureRuntimeStandup(const float theta_reference_rad) {
  if (standup_active || state::motorActive() ||
      !std::isfinite(theta_reference_rad)) {
    return false;
  }
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = theta_reference_rad;
  config.vq_limit_v = state::kMotorVectorLimitV;
  if (config.pump_v_high > config.vq_limit_v ||
      !standup_controller.configure(config)) {
    return false;
  }
  standup_config = config;
  standup_configured = true;
  return true;
}

StandupStartFailure startRuntimeStandup() {
  ensureDefaultConfig();
  if (standup_active) return StandupStartFailure::kAlreadyActive;
  if (state::motorActive()) return StandupStartFailure::kMotorActive;
  if (!realtimeReleaseReady()) return StandupStartFailure::kReleaseClock;
  if (!state::motor_config_valid) return StandupStartFailure::kMotorConfig;
  if (!state::encoder_sample_valid || !state::wheel_state.velocity_valid) {
    return StandupStartFailure::kEncoder;
  }
  if (!state::imu_ready || !state::imu_sample_valid || !state::gyro_bias_valid) {
    return StandupStartFailure::kImu;
  }
  if (!state::attitude_initialized || !state::attitude_state.valid) {
    return StandupStartFailure::kAttitude;
  }
  if (state::safety_latch.faulted()) return StandupStartFailure::kSafetyFault;

  const std::uint32_t now_us =
      static_cast<std::uint32_t>(esp_timer_get_time());
  const auto input = currentStandupInput(now_us);
  standup_controller.reset(input);
  const auto output = standup_controller.update(input);
  if (!output.valid || !std::isfinite(output.vq_v)) {
    return StandupStartFailure::kAttitude;
  }

  standup_active = true;
  state::vq_command_v = state::clampFinite(
      output.vq_v, -state::kMotorVectorLimitV, state::kMotorVectorLimitV);
  state::motor_mode = state::MotorMode::kFoc;
  return StandupStartFailure::kNone;
}

void stopRuntimeStandup() {
  if (!standup_active) return;
  standup_active = false;
  state::stopMotor();
}

void updateRuntimeStandup(const std::uint32_t now_us) {
  if (!standup_active) return;
  if (!realtimeReleaseReady()) {
    tripStandupFault(triwhirl::SafetyFault::kControlTiming);
    return;
  }
  if (state::safety_latch.faulted()) {
    standup_active = false;
    state::stopMotor();
    return;
  }
  if (state::motor_mode != state::MotorMode::kFoc) {
    standup_active = false;
    state::stopMotor();
    return;
  }
  if (!state::encoder_sample_valid || !state::wheel_state.velocity_valid) {
    tripStandupFault(triwhirl::SafetyFault::kEncoderUnavailable);
    return;
  }
  if (!state::imu_ready || !state::imu_sample_valid || !state::gyro_bias_valid ||
      !state::attitude_initialized || !state::attitude_state.valid) {
    tripStandupFault(triwhirl::SafetyFault::kImuUnavailable);
    return;
  }
  if (std::fabs(state::wheel_state.velocity_rad_s) >
      kStandupWheelHardLimitRadS) {
    tripStandupFault(triwhirl::SafetyFault::kWheelOverspeed);
    return;
  }

  const auto output = standup_controller.update(currentStandupInput(now_us));
  if (!output.valid || !std::isfinite(output.vq_v)) {
    tripStandupFault(triwhirl::SafetyFault::kInvalidNumeric);
    return;
  }
  state::vq_command_v = state::clampFinite(
      output.vq_v, -state::kMotorVectorLimitV, state::kMotorVectorLimitV);
}

bool runtimeStandupActive() { return standup_active; }

RuntimeStandupStatus runtimeStandupStatus() {
  ensureDefaultConfig();
  RuntimeStandupStatus status{};
  status.active = standup_active;
  status.config = standup_config;
  status.output = standup_controller.output();
  return status;
}

const char* standupStartFailureName(const StandupStartFailure failure) {
  switch (failure) {
    case StandupStartFailure::kNone: return "none";
    case StandupStartFailure::kAlreadyActive: return "already_active";
    case StandupStartFailure::kMotorActive: return "motor_active";
    case StandupStartFailure::kReleaseClock: return "release_clock";
    case StandupStartFailure::kMotorConfig: return "motor_config";
    case StandupStartFailure::kEncoder: return "encoder";
    case StandupStartFailure::kImu: return "imu";
    case StandupStartFailure::kAttitude: return "attitude";
    case StandupStartFailure::kSafetyFault: return "safety_fault";
  }
  return "unknown";
}

}  // namespace triwhirl::runtime
