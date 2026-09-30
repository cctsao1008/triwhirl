#pragma once

#include <cmath>

namespace triwhirl {

// Snapshot published by the motor domain for system-level control.  This type
// deliberately contains only mechanical-domain quantities and validity state;
// electrical angle, phase voltages, PWM state, and library-specific types stay
// behind the motor backend.
struct MotorControlObservation {
  float shaft_velocity_rad_s = 0.0F;
  float target_velocity_rad_s = 0.0F;
  bool initialized = false;
  bool sensor_valid = false;
  bool command_enabled = false;
  bool backend_faulted = false;
};

struct MotorControlOps {
  using BeginFn = bool (*)(void* context);
  using ServiceBackendFn = void (*)(void* context);
  using CommandTargetVelocityFn = bool (*)(void* context,
                                           float target_velocity_rad_s);
  using StopFn = void (*)(void* context);
  using ObservationFn = MotorControlObservation (*)(void* context);

  BeginFn begin = nullptr;
  ServiceBackendFn service_backend = nullptr;
  CommandTargetVelocityFn command_target_velocity = nullptr;
  StopFn stop = nullptr;
  ObservationFn observation = nullptr;
};

// Small, allocation-free system-level motor boundary.
//
// commandTargetVelocityRadS(), stop(), and observation() are the operations
// intended for the attitude/control domain.  A concrete backend must implement
// them as bounded, non-blocking command/snapshot operations when crossing task
// or core domains.
//
// serviceBackend() belongs to the motor backend's own execution domain.  It is
// explicitly separate because a backend may perform sensor-bus I/O there; the
// attitude loop must not call it merely to obtain a fresh shaft velocity.
class MotorControl {
 public:
  MotorControl() = default;

  MotorControl(void* context, const MotorControlOps& ops,
               const float target_velocity_limit_rad_s)
      : context_(context),
        ops_(ops),
        target_velocity_limit_rad_s_(target_velocity_limit_rad_s) {}

  bool valid() const {
    return context_ != nullptr && ops_.begin != nullptr &&
           ops_.service_backend != nullptr &&
           ops_.command_target_velocity != nullptr && ops_.stop != nullptr &&
           ops_.observation != nullptr &&
           std::isfinite(target_velocity_limit_rad_s_) &&
           target_velocity_limit_rad_s_ > 0.0F;
  }

  bool begin() const { return valid() && ops_.begin(context_); }

  // Motor-domain hook.  Do not call this from the attitude loop when the
  // backend can perform synchronous I/O.
  void serviceBackend() const {
    if (valid()) {
      ops_.service_backend(context_);
    }
  }

  bool commandTargetVelocityRadS(const float target_velocity_rad_s) const {
    if (!valid() || !std::isfinite(target_velocity_rad_s) ||
        std::fabs(target_velocity_rad_s) > target_velocity_limit_rad_s_) {
      return false;
    }
    return ops_.command_target_velocity(context_, target_velocity_rad_s);
  }

  void stop() const {
    if (valid()) {
      ops_.stop(context_);
    }
  }

  MotorControlObservation observation() const {
    return valid() ? ops_.observation(context_) : MotorControlObservation{};
  }

  float targetVelocityLimitRadS() const {
    return target_velocity_limit_rad_s_;
  }

 private:
  void* context_ = nullptr;
  MotorControlOps ops_{};
  float target_velocity_limit_rad_s_ = 0.0F;
};

}  // namespace triwhirl
