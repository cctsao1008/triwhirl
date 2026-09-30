# Full-Fuzzy Attitude-Control Boundary

Issue: #37

This document fixes the control-layer boundary before additional fuzzy-controller work proceeds.

## Decision

TriWhirl uses **full-fuzzy attitude control** at the system level and **latest-stable SimpleFOC** as the reaction-wheel motor-control layer.

As of 2026-09-30, the latest stable SimpleFOC release is **v2.4.0**. The repository should pin an explicit stable release/tag and must not track `master` for the target motor layer.

The supplied vendor control program is archival only. It is not a design, tuning, parity, or validation authority.

## Ownership

```text
MPU6050 / attitude estimator
        |  theta, theta_dot
        |
        |                  AS5600
        |                     |
        |                     v
        |                 SimpleFOC
        |              shaftVelocity()
        |                     |
        +----------+----------+
                   v
          Full-Fuzzy Attitude Control
                   |
                   | target_velocity [rad/s]
                   v
          SimpleFOC velocity control
                   |
             FOC / PWM / driver
                   |
                   v
                  BLDC
```

### Full-fuzzy attitude layer

Owns:

- swing-up behavior;
- capture behavior;
- near-upright balance;
- disturbance recovery;
- reaction-wheel momentum management;
- nonlinear mapping from body/wheel state to requested wheel velocity.

Required initial inputs:

- periodic body/upright angle error `theta`;
- body angular rate `theta_dot`;
- `motor.shaftVelocity()` as reaction-wheel momentum state.

Output:

- bounded `target_velocity` in rad/s.

The attitude layer does **not** emit direct `Vq`, phase voltage, duty cycle, or PWM.

### SimpleFOC motor layer

Owns:

- AS5600 integration;
- shaft angle / `shaftVelocity()`;
- motor velocity closed loop;
- motor torque/current/voltage implementation internal to SimpleFOC;
- electrical angle / FOC;
- PWM generation;
- motor-layer limits and protection supported by the selected configuration.

## Unified command interface

The only system-level actuator command is:

```text
full-fuzzy attitude controller
            |
            v
   target_velocity [rad/s]
            |
            v
  SimpleFOC velocity mode
```

Swing-up, capture, and balance are regions of the same fuzzy attitude-control problem. Do not preserve a vendor-inspired switch between voltage-mode swing control and velocity-mode balance control.

The first implementation may use separate rule groups for engineering clarity, but they must compose into one continuous wheel-velocity target with no unit-changing mode switch.

## Vendor-source independence

Do not use the vendor program for:

- attitude-control structure or gains;
- swing-up rules;
- capture thresholds or dwell times;
- SimpleFOC PI/LPF tuning;
- sensor sign conventions;
- voltage/velocity limits;
- expected dynamics;
- golden traces or pass/fail criteria.

Hardware facts must come from schematic/datasheet evidence, mechanical characterization, or our own bench measurements. Unknown values remain unknown until measured.

## Safety boundary

The following remain deterministic and outside fuzzy inference:

- sensor validity / stale-data handling;
- hard wheel-speed and target limits;
- motor / FOC fault handling;
- emergency disable;
- unrecoverable-fall protection;
- numerical sanity checks;
- minimal temporal qualification required strictly for safety.

Safety always overrides fuzzy output.

## SITL boundary

Simulation must use the same abstraction as firmware:

```text
body / wheel state
       |
       v
full-fuzzy attitude controller
       |
       v
target_velocity
       |
       v
motor velocity-servo model
       |
       v
reaction-wheel / Reuleaux plant
```

A velocity target must never be injected into the plant as though it were `Vq`.

The motor velocity-servo model should eventually be identified from our own SimpleFOC bench response rather than copied from vendor software.

## Validation direction

Validation is against explicit physical/control requirements, not vendor behavior. Key evidence includes:

- peak / RMS body-angle error;
- capture and settling time;
- peak / RMS body rate;
- peak / RMS wheel velocity;
- target-velocity continuity and boundedness;
- body- and wheel-disturbance recovery;
- momentum unloading effectiveness;
- saturation dwell;
- robustness over supported plant uncertainty;
- long-duration upright stability.

## Current migration boundary

`main` still contains native wheel kinematics and native voltage-mode FOC/PWM ownership. Those paths are not the target architecture for #37.

#39 owns the migration to the latest stable SimpleFOC motor layer. Full-fuzzy hardware integration must wait until that motor boundary is established and independently characterized.
