# SimpleFOC compatibility spike

Issue: #39

This project is an **isolated compile/link probe**. It is not the TriWhirl runtime and must not be flashed as evidence that the motor path is hardware-ready.

## Purpose

Prove that the selected toolchain can build the target motor-layer abstraction:

```text
ESP-IDF 6.1
  + Arduino-ESP32 4.0.0-rc1 as an IDF component
  + SimpleFOC v2.4.0
```

As of 2026-09-30, **SimpleFOC v2.4.0 is the latest stable release**. It is pinned explicitly; the target does not use the vendor library version and does not track SimpleFOC `master`.

A green build proves API/toolchain compatibility only. It does not prove motor behavior, tuning, or hardware readiness.

SimpleFOC v2.4.0 explicitly documents Arduino-ESP32 3.x support. Therefore the current Arduino-ESP32 4.0.0-rc1 / ESP-IDF 6.1 combination is an experiment. If it is fundamentally incompatible, adjust the Arduino/ESP-IDF integration stack while keeping the latest stable SimpleFOC release as the motor-library target.

## APIs exercised

The probe compiles and links only the architecture-relevant path:

- `MagneticSensorI2C(AS5600_I2C)`;
- `motor.linkSensor(&sensor)`;
- `motor.shaftVelocity()`;
- `BLDCMotor(...)`;
- `BLDCDriver3PWM(...)`;
- `MotionControlType::velocity`;
- `motor.init()` / `motor.initFOC()`;
- `motor.loopFOC()`;
- `motor.move(target_velocity)`.

Any numeric pin, bus, pole-pair, voltage, or velocity values in this compile-only target are placeholders unless independently supported by schematic/datasheet/measurement evidence.

No vendor PI/LPF gains, swing behavior, capture logic, sign convention, or motor limits belong in this probe.

## Local build

```bash
git submodule update --init --recursive
cd tools/simplefoc_compat
idf.py set-target esp32
idf.py build
```

Do **not** flash this compatibility application. Its `app_main()` references initialization and motor APIs only so the linker cannot optimize the compatibility surface away.

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
