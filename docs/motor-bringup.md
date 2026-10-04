# Reaction-wheel motor bring-up

This document separates the **current live legacy commissioning path** from the **target Route-B SimpleFOC motor path**. They serve different purposes and must not be treated as interchangeable control interfaces.

## Architecture status

### Current live/rollback path

The existing native ESP-IDF runtime still owns:

```text
AS5600
  -> native mechanical angle / wheel-rate estimation
  -> native sensor-based voltage-mode FOC
  -> native MCPWM 3-PWM
  -> EG2133 -> MOSFET bridge -> 2204 BLDC
```

Its higher-level commissioning actuator coordinate is measured/applied `Vq`. This path remains useful for motor/encoder bring-up, local plant identification, diagnostics, and rollback while the target motor owner is commissioned independently.

### Target path

The approved target architecture is:

```text
AS5600
  |
  v
SimpleFOC v2.4.0 sensor path
  |
  +--> shaft angle / shaftVelocity()
  +--> velocity closed loop
  +--> electrical angle / FOC / PWM
  |
  v
EG2133 -> MOSFET bridge -> BLDC
```

The system controller boundary is purely mechanical:

```text
[theta_error, theta_rate, shaftVelocity()]
                  |
                  v
         full-fuzzy attitude control
                  |
                  v
         target_velocity [rad/s]
                  |
                  v
          SimpleFOC velocity mode
```

The fuzzy/system layer must not emit `Vq`, phase voltage, duty, PWM, or SimpleFOC electrical-control types.

The current Route-B CI compiles/links this complete target-shaped software path, but the retained probe is never called by `app_main()`. That proves software integration only; it does **not** transfer AS5600/PWM ownership or authorize motor energization.

## Hardware path and currently established board mapping

The TRC board uses one 3-PWM command per bridge phase through the EG2133. The repository's board mapping preserves the current proven phase order:

```text
phase A -> GPIO33
phase B -> GPIO25
phase C -> GPIO32

AS5600 SDA -> GPIO23
AS5600 SCL -> GPIO5
AS5600 address -> 0x36
```

The PCB ties each `Moto_INx` net to both EG2133 `HINx` and active-low `LINx#`; therefore one PWM signal controls each half-bridge phase input pair.

These wiring facts are distinct from motor commissioning parameters. Pole pairs, sensor direction, electrical alignment, phase/motor polarity, safe limits, and velocity-loop tuning must still be confirmed by our own measurements before target ownership migration.

## Legacy commissioning commands

The following commands belong to the **current native commissioning/rollback runtime**:

```text
motor calibrate [amplitude_v] [electrical_hz] [turns]
motor config <pole_pairs> <sensor_dir> <offset_rad>
motor vq <volts>
motor status
motor stop

field <electrical_hz> <amplitude_v>
stop
status
telemetry [on|off]
help
```

`field` applies a bounded open-loop rotating electrical field. `motor vq` commands the native sensor-based voltage-mode FOC after valid motor electrical configuration exists.

These commands are **not** the target full-fuzzy controller API. They remain useful because the current physical plant-identification pipeline measures firmware-applied `Vq_v` as the actuator coordinate of the legacy commissioning runtime.

### Legacy automatic motor calibration

`motor calibrate` performs the native commissioning sequence:

1. apply a bounded d-axis alignment vector;
2. sweep a known electrical angle through configured electrical turns;
3. measure AS5600 multi-turn mechanical displacement;
4. derive an effective pole-pair count;
5. derive encoder/electrical direction;
6. derive electrical-angle offset;
7. install that electrical configuration for the current native runtime.

The current command has deliberately low-energy defaults:

```text
amplitude_v   0.6 V
electrical_hz 0.5 Hz
turns         4
```

Those defaults describe the existing native commissioning implementation only. They are not SimpleFOC velocity-loop tuning and are not automatically promoted into the Route-B motor configuration.

### Legacy stop semantics

The current native `stop` path commands the board's low-side zero vector. It removes commanded line-to-line voltage but is not a high-impedance physical disconnect.

This matters when deciding the target Route-B hard-safety strategy: software stop semantics must not be described as a hardware power disconnect unless the physical bridge behavior has been measured and the required safety path has been explicitly designed.

## Target SimpleFOC configuration boundary

`SimpleFocMotorBackendConfig` intentionally defaults every motor-affecting value to an invalid state. The target backend cannot initialize merely because the class was constructed.

Before a live Route-B motor-only commissioning build is allowed to start, the integration site must provide independently supported values for:

```text
I2C bus / SDA / SCL / bus rate
pole pairs
phase A/B/C GPIO order
supply voltage
motor voltage limit
sensor-alignment voltage
target-velocity limit
velocity PID P/I/D
velocity output ramp
velocity LPF Tf
```

Wiring may come from schematic/board evidence. Dynamic/tuning values must come from our own bench measurements. Do not import velocity PI/LPF values, sensor direction, limits, or expected behavior from the supplied vendor program.

The selected Route-B software stack is pinned to the reviewed Arduino-ESP32 / ESP-IDF 6.1 line plus SimpleFOC v2.4.0. `SIMPLEFOC_ESP32_USELEDC` is explicit because the SimpleFOC v2.4.0 legacy ESP32 MCPWM path depends on ESP-IDF private internals that changed in IDF 6.x. This is a software compatibility choice, not proof that LEDC timing is already suitable for balancing.

## Required motor-only bench commissioning sequence

Do not connect full-fuzzy attitude control first. Commission the target motor domain independently and record evidence in this order.

### 1. Passive sensor ownership and sign

With motor output disabled:

- prove the Route-B AS5600 path reads the physical wheel reliably;
- rotate the wheel manually in both directions;
- establish the sign convention of `shaftVelocity()`;
- record transport-error behavior and estimator noise/jitter;
- confirm there is no parallel target-path wheel estimator reading the same sensor.

Acceptance is a stable mechanical observation with an explicitly documented sign convention. No motor energization is required for this step.

### 2. Phase order, pole pairs, and electrical alignment

At a deliberately bounded commissioning voltage:

- independently establish pole pairs;
- confirm phase order / driver polarity;
- run SimpleFOC electrical alignment;
- verify the sensor/motor direction relationship;
- confirm initialization failure always leaves the actuator disabled.

Do not assume the currently encoded pole-pair or alignment values are authoritative merely because historical/native firmware used them.

### 3. LEDC PWM characterization

Measure the actual selected PWM backend on the target build:

- PWM frequency;
- effective duty resolution;
- update behavior/jitter;
- three-phase duty relationship;
- stop/disable waveform behavior.

The Route-B compile gate proves API compatibility only; it does not establish these physical timing properties.

### 4. Safe actuation envelope

Before velocity tuning, establish conservative physical bounds:

- supply voltage during test;
- motor voltage limit;
- current/temperature behavior even though there is no phase-current feedback loop;
- safe wheel-speed envelope;
- mechanical containment and overspeed stop criterion.

A software voltage limit is not a substitute for verifying current, thermal, and mechanical behavior.

### 5. Velocity estimator and closed-loop response

Characterize SimpleFOC velocity mode independently of the body controller. Required observables include at least:

```text
target_velocity [rad/s]
shaftVelocity() [rad/s]
command generation / age
actuator enabled state
sensor-valid / backend-fault state
service timing
```

Use bounded target steps in both directions and record:

- deadband / minimum controllable speed;
- rise and settling time;
- overshoot;
- steady-state error;
- acceleration/slew saturation;
- reversal behavior;
- estimator noise and lag;
- behavior near the intended speed limit.

Choose `PID_velocity`, output ramp, velocity LPF, and target limit from this evidence. The repository intentionally has no production defaults for these values yet.

### 6. Stop and hard-safety decision

Measure, rather than infer, the Route-B stop path:

```text
command/stop publication
  -> motor executor acceptance
  -> backend service
  -> SimpleFOC disable / PWM state
  -> physical de-energized bridge behavior
```

Software already measures command publication -> executor acceptance and motor-task scheduling/service timing. Hardware work must determine the meaningful stop-to-PWM / stop-to-de-energized latency and whether that bound is sufficient.

If the normal motor task/mailbox path is not sufficient for required fault response, define an independent hard-shutdown owner. Do not call the normal task path "hard safety" merely because it fails closed in host tests.

## Relationship to local plant identification

Issue #12 currently identifies the near-upright physical plant using measured legacy firmware `Vq_v`:

```text
x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
u = measured firmware Vq_v
```

That is valid for the present commissioning actuator coordinate. It does **not** imply that the future fuzzy controller outputs Vq.

After the Route-B motor velocity loop is physically characterized, identify the mechanical target-velocity servo behavior separately:

```text
target_velocity [rad/s]
        -> measured wheel velocity / acceleration
```

Only then should local-plant evidence and motor-servo evidence be combined for physically supported full-fuzzy normalization/rule/robustness work.

## Ownership migration gate

Do not retire the native motor path until the motor-only Route-B evidence above is complete.

The migration sequence remains:

1. SimpleFOC becomes the target-path AS5600 owner;
2. target runtime exposes `shaftVelocity()` through the mechanical observation snapshot;
3. system commands are only bounded `target_velocity [rad/s]`;
4. SimpleFOC owns velocity control, electrical angle, FOC and PWM;
5. native target-path wheel estimator / voltage-mode FOC / direct PWM ownership is removed only after the Route-B path is independently commissioned;
6. current native code may remain temporarily as explicitly selected rollback/commissioning scaffolding, but it must not run in parallel with the target motor owner.

## Current authorization boundary

At the present repository state:

- compile/link architecture: **proven in CI**;
- host mailbox/executor fail-closed semantics: **proven in deterministic tests**;
- current native Vq commissioning path: **available**;
- live Route-B AS5600 ownership: **not migrated**;
- SimpleFOC PI/LPF/output-ramp commissioning: **not measured**;
- LEDC motor-loop timing: **not measured**;
- physical stop/de-energize latency: **not measured**;
- full-fuzzy hardware balance: **blocked**.

No balancing or autonomous stand-up hardware trial should be started from the inactive Route-B compile result alone.
