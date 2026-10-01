#pragma once

#include <cmath>
#include <cstdint>

#include "triwhirl/motor_control.hpp"
#include "triwhirl/motor_mailbox.hpp"

namespace triwhirl {

struct MotorExecutionConfig {
  // Zero disables command-age timeout. A nonzero value is an execution-domain
  // safety policy, not a motor tuning parameter. Keep it below 2^31 us so
  // uint32_t modular elapsed-time arithmetic remains unambiguous.
  uint32_t command_timeout_us = 0U;
};

struct MotorExecutionStats {
  uint32_t service_calls = 0U;
  uint32_t applied_commands = 0U;
  uint32_t applied_stops = 0U;
  uint32_t timeout_stops = 0U;
  uint32_t safety_stops = 0U;
  uint32_t backend_faults = 0U;
};

// Motor-side executor for the cross-domain contract.
//
// The attitude domain publishes latest-value commands to MotorCommandMailbox.
// Exactly one motor-domain task owns this object and the concrete MotorControl
// backend. service() consumes the newest command, performs all backend/SimpleFOC
// calls in the motor domain, and publishes a coherent mechanical observation.
//
// No task creation or scheduling policy lives here. That separation allows the
// ordering/fail-safe semantics to be exhaustively host-tested before a FreeRTOS
// wrapper is introduced. A stop request is physically observable as disabled
// after at most one normal service() invocation, provided the backend honors its
// own stop + service contract. A detected backend/sensor inconsistency may cause
// one additional bounded backend service in the same invocation to force stop.
class MotorExecutionDomain {
 public:
  MotorExecutionDomain(const MotorControl& motor_control,
                       MotorCommandMailbox* command_mailbox,
                       MotorObservationMailbox* observation_mailbox,
                       const MotorExecutionConfig& config = {})
      : motor_control_(motor_control),
        command_mailbox_(command_mailbox),
        observation_mailbox_(observation_mailbox),
        config_(config) {}

  bool valid() const {
    return motor_control_.valid() && command_mailbox_ != nullptr &&
           observation_mailbox_ != nullptr &&
           (config_.command_timeout_us == 0U ||
            config_.command_timeout_us < 0x80000000U);
  }

  bool begin(const uint32_t now_us32) {
    if (!valid()) {
      backend_fault_latched_ = true;
      publishSyntheticFault(now_us32);
      return false;
    }

    started_ = motor_control_.begin();
    if (!started_) {
      backend_fault_latched_ = true;
      ++stats_.backend_faults;
      publishSyntheticFault(now_us32);
      return false;
    }

    // Establish a known de-energized baseline in the motor domain. Some
    // backends make stop() a cached request and enact it in serviceBackend().
    motor_control_.stop();
    desired_enabled_ = false;
    applied_target_velocity_rad_s_ = 0.0F;
    motor_control_.serviceBackend();
    ++stats_.service_calls;

    MotorControlObservation observation = motor_control_.observation();
    if (!observationFinite(observation) || observation.backend_faulted ||
        observation.actuator_enabled) {
      latchBackendFault();
      forceStopAndService();
      observation = motor_control_.observation();
    }
    publishObservation(observation, now_us32);
    return !backend_fault_latched_;
  }

  void service(const uint32_t now_us32) {
    if (!valid() || !started_) {
      backend_fault_latched_ = true;
      publishSyntheticFault(now_us32);
      return;
    }

    MotorCommandSnapshot command{};
    const bool have_command = command_mailbox_->tryRead(&command);
    const bool new_command =
        have_command && command.generation != last_seen_command_generation_;

    if (new_command) {
      last_seen_command_generation_ = command.generation;
      command_timed_out_ = false;
      if (!command.enabled) {
        applyStopCommand(command, now_us32);
      } else if (commandExpired(command, now_us32)) {
        applyTimedOutCommand(command, now_us32);
      } else if (!backend_fault_latched_) {
        applyTargetCommand(command, now_us32);
      }
    } else if (desired_enabled_ && commandTimeoutEnabled() && have_command &&
               commandExpired(command, now_us32)) {
      // The last accepted target has gone stale because the attitude domain has
      // not refreshed the latest-value command within the configured policy.
      motor_control_.stop();
      desired_enabled_ = false;
      applied_target_velocity_rad_s_ = 0.0F;
      command_timed_out_ = true;
      ++stats_.timeout_stops;
    }

    // This is the normal motor-domain service point. It may perform synchronous
    // sensor I/O (for example AS5600) and FOC work; the attitude task never
    // enters it.
    motor_control_.serviceBackend();
    ++stats_.service_calls;

    MotorControlObservation observation = motor_control_.observation();
    bool need_fail_safe_stop = false;

    if (!observationFinite(observation) || observation.backend_faulted) {
      latchBackendFault();
      need_fail_safe_stop = true;
    }
    if (observation.actuator_enabled && !desired_enabled_) {
      latchBackendFault();
      need_fail_safe_stop = true;
    }
    if (observation.actuator_enabled && !observation.sensor_valid) {
      // Sensor invalidity is propagated distinctly from a backend fault, but a
      // closed-loop actuator is never allowed to remain energized on it.
      need_fail_safe_stop = true;
    }

    if (need_fail_safe_stop) {
      forceStopAndService();
      observation = motor_control_.observation();
      if (!observationFinite(observation) || observation.backend_faulted ||
          observation.actuator_enabled) {
        latchBackendFault();
      }
    }

    publishObservation(observation, now_us32);
  }

  bool started() const { return started_; }
  bool backendFaultLatched() const { return backend_fault_latched_; }
  bool commandTimedOut() const { return command_timed_out_; }
  uint32_t appliedCommandGeneration() const {
    return applied_command_generation_;
  }
  const MotorExecutionStats& stats() const { return stats_; }

 private:
  static uint32_t elapsedUs32(const uint32_t now_us32,
                              const uint32_t then_us32) {
    return now_us32 - then_us32;
  }

  bool commandTimeoutEnabled() const {
    return config_.command_timeout_us != 0U;
  }

  bool commandExpired(const MotorCommandSnapshot& command,
                      const uint32_t now_us32) const {
    return commandTimeoutEnabled() &&
           elapsedUs32(now_us32, command.issued_at_us32) >
               config_.command_timeout_us;
  }

  static bool observationFinite(const MotorControlObservation& observation) {
    return std::isfinite(observation.shaft_velocity_rad_s) &&
           std::isfinite(observation.target_velocity_rad_s);
  }

  void latchBackendFault() {
    if (!backend_fault_latched_) {
      ++stats_.backend_faults;
    }
    backend_fault_latched_ = true;
  }

  void applyTargetCommand(const MotorCommandSnapshot& command,
                          const uint32_t now_us32) {
    if (!motor_control_.commandTargetVelocityRadS(
            command.target_velocity_rad_s)) {
      latchBackendFault();
      motor_control_.stop();
      desired_enabled_ = false;
      applied_target_velocity_rad_s_ = 0.0F;
      return;
    }

    desired_enabled_ = true;
    applied_target_velocity_rad_s_ = command.target_velocity_rad_s;
    applied_command_generation_ = command.generation;
    command_apply_latency_us_ =
        elapsedUs32(now_us32, command.issued_at_us32);
    ++stats_.applied_commands;
  }

  void applyStopCommand(const MotorCommandSnapshot& command,
                        const uint32_t now_us32) {
    motor_control_.stop();
    desired_enabled_ = false;
    applied_target_velocity_rad_s_ = 0.0F;
    applied_command_generation_ = command.generation;
    command_apply_latency_us_ =
        elapsedUs32(now_us32, command.issued_at_us32);
    ++stats_.applied_stops;
  }

  void applyTimedOutCommand(const MotorCommandSnapshot& command,
                            const uint32_t now_us32) {
    motor_control_.stop();
    desired_enabled_ = false;
    applied_target_velocity_rad_s_ = 0.0F;
    applied_command_generation_ = command.generation;
    command_apply_latency_us_ =
        elapsedUs32(now_us32, command.issued_at_us32);
    command_timed_out_ = true;
    ++stats_.timeout_stops;
  }

  void forceStopAndService() {
    motor_control_.stop();
    desired_enabled_ = false;
    applied_target_velocity_rad_s_ = 0.0F;
    motor_control_.serviceBackend();
    ++stats_.service_calls;
    ++stats_.safety_stops;
  }

  void publishSyntheticFault(const uint32_t now_us32) {
    if (observation_mailbox_ == nullptr) {
      return;
    }
    observation_mailbox_->publish(
        0.0F, 0.0F, applied_command_generation_, command_apply_latency_us_,
        now_us32, false, false, true, false, command_timed_out_);
  }

  void publishObservation(const MotorControlObservation& observation,
                          const uint32_t now_us32) {
    const bool finite = observationFinite(observation);
    const float shaft_velocity =
        finite ? observation.shaft_velocity_rad_s : 0.0F;
    const bool sensor_valid = finite && observation.sensor_valid;
    const bool backend_faulted =
        backend_fault_latched_ || observation.backend_faulted || !finite;

    observation_mailbox_->publish(
        shaft_velocity, applied_target_velocity_rad_s_,
        applied_command_generation_, command_apply_latency_us_, now_us32,
        observation.initialized, sensor_valid, backend_faulted,
        observation.actuator_enabled, command_timed_out_);
  }

  MotorControl motor_control_{};
  MotorCommandMailbox* command_mailbox_ = nullptr;
  MotorObservationMailbox* observation_mailbox_ = nullptr;
  MotorExecutionConfig config_{};
  MotorExecutionStats stats_{};

  bool started_ = false;
  bool backend_fault_latched_ = false;
  bool desired_enabled_ = false;
  bool command_timed_out_ = false;
  uint32_t last_seen_command_generation_ = 0U;
  uint32_t applied_command_generation_ = 0U;
  uint32_t command_apply_latency_us_ = 0U;
  float applied_target_velocity_rad_s_ = 0.0F;
};

}  // namespace triwhirl
