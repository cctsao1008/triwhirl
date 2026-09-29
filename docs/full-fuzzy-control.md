# Full-Fuzzy Control Boundary

Issue: #37

This document fixes the control-layer boundary before additional fuzzy-controller work proceeds.

## Decision

`Full fuzzy` applies to the **system-level standup, capture and balance control laws**. It does not replace the motor-control stack.

SimpleFOC is the intended owner of the reaction-wheel motor path:

```text
AS5600
  |
  v
SimpleFOC sensor path
  |
  +--> shaft angle / shaftVelocity()
  |
  +--> velocity PI when MotionControlType::velocity is selected
  |
  +--> electrical angle / FOC / PWM
  |
  v
EG2133 -> BLDC
```

The body controller receives MPU6050-derived body state plus the SimpleFOC wheel velocity state.

## Balance interface

```text
theta, theta_dot, motor.shaftVelocity()
                |
                v
          Fuzzy Balance
                |
                v
         target_velocity
                |
                v
SimpleFOC MotionControlType::velocity
                |
                v
          motor.move(target_velocity)
```

The fuzzy controller replaces the state-feedback/LQR mapping only. SimpleFOC remains responsible for closing the wheel-speed servo loop.

## Swing-up interface

```text
theta, theta_dot, wheel state
                |
                v
           Fuzzy Swing
                |
                v
          voltage command
                |
                v
SimpleFOC MotionControlType::torque
     TorqueControlType::voltage
                |
                v
          motor.move(voltage)
```

The source-aligned baseline used voltage-mode torque control for swing-up and velocity control for balance. These commands have different dimensions and must not be numerically blended.

## Capture / transition

Fuzzy inference may generate capture confidence or balance authority, but a deterministic temporal supervisor owns mode qualification, hysteresis, dwell time, recovery and bumpless transfer.

```text
fuzzy capture confidence
          |
          v
deterministic temporal supervisor
          |
     +----+----+
     |         |
   Swing     Balance
```

## Momentum management

Wheel velocity is mandatory state for the fuzzy balance controller. The rule base must explicitly unload accumulated reaction-wheel momentum rather than merely stabilize body angle while wheel speed drifts toward saturation.

The vendor-aligned baseline adjusted the balance reference after sustained stable operation when wheel speed remained non-zero. The full-fuzzy implementation may realize momentum unloading differently, but the physical objective must remain explicit and testable.

## Safety boundary

The following remain deterministic and outside fuzzy inference:

- sensor validity and timeout handling;
- hard wheel-speed / voltage / target limits;
- motor / FOC fault handling;
- emergency disable and unrecoverable-fall guards;
- temporal mode qualification needed to prevent chatter.

## SITL parity rule

Simulation must use the same command semantics as firmware.

- Balance: fuzzy `target_velocity` must pass through a SimpleFOC-equivalent velocity-servo layer before the plant sees voltage / torque authority.
- Swing: fuzzy voltage command may drive the voltage-mode actuator model directly.
- A velocity target must never be injected into a plant as though it were `Vq`.

## Current migration boundary

At the time this document was added, `main` still contains native AS5600 kinematics, native voltage-mode FOC and native PWM ownership. Those implementations are useful historical / commissioning work but are not the target motor-control ownership for #37.

The next firmware milestone is therefore to restore and validate SimpleFOC ownership before wiring the full-fuzzy controller into hardware.

The supplied vendor archive uses SimpleFOC 2.1.1. A newer SimpleFOC release may be evaluated, but any version change must be explicit and must preserve or re-validate sensor direction, shaft-velocity sign, motion-mode semantics, velocity-loop behavior and PWM behavior.
