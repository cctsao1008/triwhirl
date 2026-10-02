# SimpleFOC compatibility probe

Issues: #39, #43, #53

This directory is an **isolated upstream API/toolchain compile/link probe**. It is not the TriWhirl runtime and must not be flashed as evidence that the motor path is hardware-ready.

## Purpose

Keep one minimal check for the selected dependency stack itself:

```text
pioarduino / PlatformIO-compatible build
  + Arduino-ESP32 4.0.0-RC1
  + ESP-IDF 6.1
  + SimpleFOC v2.4.0
```

The stack remains pinned explicitly for deterministic CI. A green build proves API/toolchain compatibility only; it does not prove motor behavior, tuning, scheduling, hardware readiness, or production ownership.

The production-shaped `MotorControl` backend no longer lives under `tools/`. It is compiled by the Route-B firmware graph from:

```text
components/triwhirl_simplefoc
```

That component is the integration authority for TriWhirl's eventual SimpleFOC motor owner. This compatibility project deliberately goes back to a small raw SimpleFOC API probe so there is no second backend implementation to drift.

## ESP32 PWM backend on IDF 6.1

SimpleFOC v2.4.0's legacy ESP32 MCPWM path depends on ESP-IDF 5.x private MCPWM internals that changed in IDF 6.1. TriWhirl does not copy or locally redefine those private structures merely to make the build pass.

Both this probe and the Route-B firmware graph therefore select SimpleFOC's supported LEDC backend with:

```text
SIMPLEFOC_ESP32_USELEDC
```

This is a build/integration decision only. PWM frequency, duty behavior and motor-loop performance remain measurement gates before any balancing trial.

## Dependency policy

The probe does **not** vendor SimpleFOC as a Git submodule. PlatformIO resolves the exact reviewed commits through `platformio.ini`:

- pioarduino `platform-espressif32`: `fabd8cd10c7f05672832b405dea91d38b6e6e8a0`;
- Arduino-ESP32 4.0.0-RC1: `76f683d935b390f2805301ec10a5562bbbb37811`;
- SimpleFOC v2.4.0: `4f072b365f6e0185adca544071e595834405babc`.

"Track latest" means reviewing and explicitly updating those pins when a newer selected Arduino-ESP32 4.x or stable SimpleFOC release is adopted. CI must not float on `master`/`develop`.

## APIs exercised

The isolated probe keeps the architecture-relevant upstream API linkable:

- `MagneticSensorI2C(AS5600_I2C)`;
- `motor.linkSensor(&sensor)`;
- `motor.shaftVelocity()`;
- `BLDCMotor(...)`;
- `BLDCDriver3PWM(...)`;
- `MotionControlType::velocity`;
- `motor.init()` / `motor.initFOC()` / `motor.enable()` / `motor.disable()` / `motor.loopFOC()`;
- `motor.move(target_velocity)`.

Any numeric pin, pole-pair, voltage or velocity value in this directory is a compile-only placeholder. No vendor PI/LPF gains, sign convention, swing behavior, capture logic or motor limits are design authority.

## Local build

```bash
pio run -d tools/simplefoc_compat
```

Do **not** flash this compatibility application.

## Production boundary

The production target architecture remains:

```text
full-fuzzy attitude control
        |
        | target_velocity_rad_s
        v
MotorControl / mailbox boundary
        |
        v
MotorExecutionDomain + pinned motor task
        |
        v
components/triwhirl_simplefoc
        |
        +-- AS5600
        +-- velocity servo
        +-- FOC / PWM
```

The Route-B production-shaped backend is still inactive in `app_main()`. AS5600/PWM ownership transfer and hardware commissioning are later gates.
