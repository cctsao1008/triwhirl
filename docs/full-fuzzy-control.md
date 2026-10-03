# Full-fuzzy attitude control over the SimpleFOC motor boundary

## Status

This document defines the **target control architecture** for TriWhirl.

It does not claim that the live hardware runtime has already completed the motor-ownership migration. The current runtime still contains the older native AS5600 / wheel-kinematics / direct-Vq / MCPWM path as commissioning and rollback scaffolding while the new motor boundary is proven incrementally.

The supplied vendor program is archival material only. It is not a design, tuning, sign, limit, behavior, or validation authority for this architecture.

## Target system boundary

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

The only system-level actuator request produced by the attitude controller is a bounded wheel target velocity in rad/s.

The fuzzy layer never emits direct `Vq`, phase voltage, duty cycle, or PWM.

## Ownership

### Full-fuzzy attitude layer

The attitude layer owns the nonlinear system-level mapping from body/wheel state to wheel target velocity, including:

- swing-up behavior;
- capture behavior;
- near-upright balance;
- disturbance recovery;
- reaction-wheel momentum management.

Required initial state inputs are:

```text
theta_error
theta_dot
wheel_velocity
```

Derived state such as normalized energy or momentum/saturation margin may be added only when supported by model or measured evidence.

### SimpleFOC motor layer

The target motor layer owns:

- AS5600 integration;
- shaft angle and `shaftVelocity()`;
- wheel velocity regulation;
- electrical-angle handling;
- FOC;
- PWM generation;
- motor-layer limits/protection supported by the selected configuration.

The project currently pins the reviewed latest-stable SimpleFOC release rather than tracking upstream `master`. Version advancement is an explicit review/CI event.

### Deterministic safety

Safety remains outside fuzzy inference and always overrides fuzzy intent. Examples include:

- stale or invalid sensor state;
- non-finite numerics;
- hard wheel-speed / target limits;
- motor/backend faults;
- emergency disable;
- unrecoverable-fall policy;
- initialization/alignment failure handling.

## Execution-domain boundary

The attitude-control domain and motor/SimpleFOC domain communicate through a mechanical-domain contract rather than SimpleFOC types:

```text
attitude/control domain                 motor/SimpleFOC domain
-----------------------                 ----------------------
read shaft snapshot            <-----   publish shaftVelocity
write target_velocity_rad_s    ----->   consume latest target
                                       AS5600 / loopFOC / move
```

`triwhirl_core` remains independent of Arduino and SimpleFOC APIs.

The repository already has host-tested latest-value command/snapshot transport, fail-closed execution semantics, and a target-side scheduling wrapper. The production-shaped SimpleFOC backend also compiles/links in the pinned Route-B graph without being started by `app_main()`.

Those software proofs do **not** authorize motor hardware operation or prove physical command-to-PWM timing.

## SITL command semantics

Simulation follows the same unit boundary as the target firmware:

```text
body / wheel state
       |
       v
full-fuzzy attitude controller
       |
       v
target_velocity [rad/s]
       |
       v
provisional / identified motor velocity servo
       |
       v
wheel_accel_command [rad/s^2]
       |
       v
reaction-wheel / Reuleaux plant
```

A target velocity is never injected into a Vq-input plant as though the units were interchangeable.

For the existing provisional near-upright B/C models, the repository now has a simulation-only algebraic transform between the historical Vq coordinate and commanded wheel acceleration. The latent `Vq_equiv` produced by that transform exists only to preserve the identified local equations; it is not a production command path.

## Model authority

The near-upright B/C models are provisional local evidence. Passing their deterministic regression or actuator-coordinate parity gates does not make them a validated digital twin.

The geometry-derived full-swing model remains exploratory. Its explicit authority is:

```text
validation_authority=NONE
```

Do not tune or validate a global fuzzy swing-up law as though the far-field model were measured truth.

## Fuzzy inference foundation

The firmware-side fuzzy foundation is allocation-free and currently provides:

- a symmetric five-term NL/NS/ZE/PS/PL membership partition over normalized `[-1,+1]`;
- zero-order Takagi-Sugeno 1-D inference;
- zero-order Takagi-Sugeno 2-D inference;
- zero-order Takagi-Sugeno 3-D inference;
- weighted-average defuzzification.

The controller owns normalization, rule surfaces, output scaling, and physical limits. Generic inference primitives do not embed motor tuning or plant-specific gains.

`fuzzylite` may be used as optional host/reference tooling if useful, but it is not a required firmware dependency.

## Vendor-source independence

Do not derive the target design from the supplied vendor program for:

- LQR/state-feedback structure or gains;
- swing-up law;
- capture thresholds/timing;
- SimpleFOC PI/LPF values;
- sensor sign conventions;
- voltage/velocity limits;
- expected dynamic behavior;
- SITL golden traces or pass/fail criteria.

Hardware facts come from schematics, component datasheets, mechanical characterization, or independent measurement. Unknown values remain unknown until measured.

## Validation direction

Near-upright fuzzy development should be judged on explicit physical/control metrics such as:

- peak and RMS body-angle error;
- body-rate envelope;
- settling/recovery time;
- peak/RMS wheel velocity;
- wheel-speed saturation dwell;
- target-velocity continuity and saturation;
- reaction-wheel momentum unloading;
- body- and wheel-disturbance recovery;
- mirrored-state symmetry where the local model is symmetric;
- robustness across B/C plant variation and a stated motor-servo uncertainty envelope;
- long-duration bounded behavior.

Hardware balancing remains blocked until software/simulation gates are strong enough and independent motor commissioning establishes the required physical parameters and actuator behavior.

## Migration order

The intended sequence is:

1. keep the generic fuzzy inference core deterministic and tested;
2. keep `target_velocity` / motor-servo / local-plant units explicit in SITL;
3. implement and validate near-upright fuzzy balance on the provisional local uncertainty set;
4. independently commission the SimpleFOC motor velocity servo;
5. replace provisional servo fixtures with identified motor behavior and uncertainty;
6. transfer target runtime AS5600/FOC/PWM ownership to SimpleFOC only after the motor execution-domain gates are satisfied;
7. connect the validated fuzzy attitude output through the `MotorControl` boundary;
8. extend fuzzy control through capture and swing-up only when the corresponding plant/model evidence earns sufficient authority.

No step in this sequence requires restoring the vendor controller or creating a fuzzy-to-`Vq` motor path.
