#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include "triwhirl/fuzzy_balance_controller.hpp"
#include "triwhirl/motor_mailbox.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace triwhirl {

struct FuzzyAttitudeCommandConfig {
  FuzzyBalanceConfig fuzzy{};
  float upright_reference_rad = std::numeric_limits<float>::quiet_NaN();

  // Maximum age of the motor-domain observation consumed by the attitude
  // domain. Deliberately invalid by default; must remain below half the uint32
  // time range so modular elapsed-time arithmetic is unambiguous.
  std::uint32_t max_motor_observation_age_us = 0U;
};

inline bool validFuzzyAttitudeCommandConfig(
    const FuzzyAttitudeCommandConfig& config) {
  return validFuzzyBalanceConfig(config.fuzzy) &&
         std::isfinite(config.upright_reference_rad) &&
         config.max_motor_observation_age_us > 0U &&
         config.max_motor_observation_age_us < 0x80000000U;
}

struct FuzzyAttitudeCommandInput {
  float theta_rad = std::numeric_limits<float>::quiet_NaN();
  float theta_rate_rad_s = std::numeric_limits<float>::quiet_NaN();
  bool attitude_valid = false;

  std::uint32_t now_us32 = 0U;
  bool motor_observation_available = false;
  MotorObservationSnapshot motor{};
};

enum class FuzzyAttitudeCommandStopReason : std::uint8_t {
  kNone = 0,
  kInvalidConfig,
  kInvalidAttitude,
  kMotorObservationUnavailable,
  kMotorNotInitialized,
  kMotorSensorInvalid,
  kMotorBackendFault,
  kMotorCommandTimedOut,
  kMotorObservationStale,
  kInvalidFuzzyOutput,
};

struct FuzzyAttitudeCommandOutput {
  bool target_valid = false;
  FuzzyAttitudeCommandStopReason stop_reason =
      FuzzyAttitudeCommandStopReason::kInvalidConfig;

  float theta_error_rad = 0.0F;
  float target_velocity_rad_s = 0.0F;
  std::uint32_t motor_observation_age_us = 0U;
  FuzzyBalanceOutput fuzzy{};
};

// Allocation-free production-shaped adapter from global body attitude and the
// motor-domain mechanical observation to the pure-fuzzy target-velocity
// command boundary. It performs no motor-library calls and owns no actuator.
class FuzzyAttitudeCommandController {
 public:
  explicit FuzzyAttitudeCommandController(
      const FuzzyAttitudeCommandConfig& config)
      : config_(config), fuzzy_(config.fuzzy) {}

  bool valid() const { return validFuzzyAttitudeCommandConfig(config_); }

  FuzzyAttitudeCommandOutput evaluate(
      const FuzzyAttitudeCommandInput& input) const {
    FuzzyAttitudeCommandOutput output{};

    if (!valid()) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kInvalidConfig;
      return output;
    }

    if (!input.attitude_valid || !std::isfinite(input.theta_rad) ||
        !std::isfinite(input.theta_rate_rad_s)) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kInvalidAttitude;
      return output;
    }

    if (!input.motor_observation_available) {
      output.stop_reason =
          FuzzyAttitudeCommandStopReason::kMotorObservationUnavailable;
      return output;
    }
    if (!input.motor.initialized) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kMotorNotInitialized;
      return output;
    }
    if (!input.motor.sensor_valid ||
        !std::isfinite(input.motor.shaft_velocity_rad_s)) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kMotorSensorInvalid;
      return output;
    }
    if (input.motor.backend_faulted) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kMotorBackendFault;
      return output;
    }
    if (input.motor.command_timed_out) {
      output.stop_reason =
          FuzzyAttitudeCommandStopReason::kMotorCommandTimedOut;
      return output;
    }

    output.motor_observation_age_us =
        input.now_us32 - input.motor.service_start_us32;
    if (output.motor_observation_age_us >
        config_.max_motor_observation_age_us) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kMotorObservationStale;
      return output;
    }

    output.theta_error_rad =
        periodicUprightErrorRad(input.theta_rad, config_.upright_reference_rad);
    if (!std::isfinite(output.theta_error_rad)) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kInvalidAttitude;
      return output;
    }

    output.fuzzy = fuzzy_.evaluate(FuzzyBalanceInput{
        output.theta_error_rad, input.theta_rate_rad_s,
        input.motor.shaft_velocity_rad_s});
    if (!output.fuzzy.valid ||
        !std::isfinite(output.fuzzy.target_velocity_rad_s)) {
      output.stop_reason = FuzzyAttitudeCommandStopReason::kInvalidFuzzyOutput;
      return output;
    }

    output.target_velocity_rad_s = output.fuzzy.target_velocity_rad_s;
    output.target_valid = true;
    output.stop_reason = FuzzyAttitudeCommandStopReason::kNone;
    return output;
  }

  // Publish the fail-closed decision through the existing latest-value motor
  // mailbox. A rejected state always emits an explicit stop request; a valid
  // state emits only the bounded target produced by FuzzyBalanceController.
  std::uint32_t publish(const FuzzyAttitudeCommandInput& input,
                        MotorCommandMailbox* command_mailbox) const {
    if (command_mailbox == nullptr) return 0U;
    const FuzzyAttitudeCommandOutput output = evaluate(input);
    if (!output.target_valid) {
      return command_mailbox->publishStop(input.now_us32);
    }
    return command_mailbox->publishTarget(output.target_velocity_rad_s,
                                          input.now_us32);
  }

 private:
  FuzzyAttitudeCommandConfig config_{};
  FuzzyBalanceController fuzzy_;
};

}  // namespace triwhirl
