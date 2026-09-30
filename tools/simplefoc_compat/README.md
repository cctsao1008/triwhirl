# SimpleFOC compatibility spike

Issue: #39

This project is an **isolated compile/link probe**. It is not the TriWhirl runtime and must not be flashed as evidence that the motor path is hardware-ready.

## Purpose

Prove that the selected Route-B toolchain can build the target motor-layer abstraction:

```text
pioarduino / PlatformIO-compatible build
  + Arduino-ESP32 4.0.0-RC1
  + ESP-IDF 6.1
  + SimpleFOC v2.4.0
```

As of 2026-10-01, **SimpleFOC v2.4.0 is the latest stable release** and Arduino-ESP32 **4.0.0-RC1** is the current 4.x release candidate based on ESP-IDF 6.1. Both are pinned explicitly for deterministic CI.

A green build proves API/toolchain compatibility only. It does not prove motor behavior, tuning, or hardware readiness.

SimpleFOC v2.4.0 explicitly documents Arduino-ESP32 3.x compatibility. Therefore Arduino-ESP32 4.x / ESP-IDF 6.1 remains an intentional compatibility experiment. We will not downgrade SimpleFOC to recover compatibility; any necessary adaptation belongs in the Arduino/pioarduino integration layer or a narrow TriWhirl compatibility shim.

## Dependency policy

The probe intentionally does **not** vendor SimpleFOC as a Git submodule. PlatformIO resolves the pinned SimpleFOC commit through `lib_deps`.

The `platformio.ini` pins:

- pioarduino `platform-espressif32` IDF-6 preparation commit;
- Arduino-ESP32 4.0.0-RC1 exact commit;
- SimpleFOC v2.4.0 exact release commit.

"Track latest" means explicitly reviewing and updating these pins when a newer Arduino-ESP32 4.x RC/stable or newer stable SimpleFOC release is selected. It does not mean floating `master`/`develop` dependencies in CI.

## APIs exercised

The probe compiles and links only the architecture-relevant path:

- `MagneticSensorI2C(AS5600_I2C)`;
- `motor.linkSensor(&sensor)`;
- `motor.shaftVelocity()`;
- `BLDCMotor(...)`;
- `BLDCDriver3PWM(...)`;
- `MotionControlType::velocity`;
- `motor.init()` / `motor.initFOC()` / `motor.loopFOC()`;
- `motor.move(target_velocity)`.

Any numeric pin, bus, pole-pair, voltage, or velocity values in this compile-only target are placeholders unless independently supported by schematic/datasheet/measurement evidence.

No vendor PI/LPF gains, swing behavior, capture logic, sign convention, or motor limits belong in this probe.

## Local build

Install the pioarduino-compatible PlatformIO core using the pioarduino installer, then run:

```bash
pio run -d tools/simplefoc_compat
```

Do **not** flash this compatibility application. It exists only to compile and link the selected API surface.

## Gate after a successful build

A green build permits only the next software step: introduce a narrow SimpleFOC motor adapter in the real runtime.

Before any balancing hardware trial, independently characterize:

- AS5600 direction and wheel-speed sign;
- motor pole pairs;
- 3-PWM phase order and driver polarity;
- electrical alignment behavior;
- safe supply/motor voltage and current limits;
- safe wheel-speed envelope;
- SimpleFOC velocity-loop step response, bandwidth, overshoot, and saturation;
- velocity-estimator noise and latency.

The full-fuzzy attitude controller will consume `shaftVelocity()` and emit only bounded `target_velocity` in rad/s. The motor velocity servo, FOC, and PWM remain SimpleFOC responsibilities.
