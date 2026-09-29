# SimpleFOC compatibility spike

Issue: #39

This project is an **isolated compile/link probe**. It is not the TriWhirl runtime and must not be flashed as evidence that the motor path is hardware-ready.

## Purpose

Prove that the source-aligned motor ownership can coexist with the current toolchain before the native TriWhirl motor path is changed:

```text
ESP-IDF 6.1
  + Arduino-ESP32 4.0.0-rc1 as an IDF component
  + SimpleFOC 2.4.0 (git submodule, pinned commit)
```

The vendor behavioral reference remains SimpleFOC 2.1.1. Passing this build proves API/toolchain compatibility only; it does not prove behavioral parity.

## APIs exercised

The probe compiles and links the source-aligned path:

- `MagneticSensorI2C(AS5600_I2C)`;
- dedicated `TwoWire(1)` on SDA 23 / SCL 5;
- `motor.linkSensor(&sensor)`;
- `motor.shaftVelocity()`;
- `BLDCMotor(7)`;
- `BLDCDriver3PWM(33, 25, 32)`;
- `MotionControlType::torque` + `TorqueControlType::voltage` for swing-up;
- `MotionControlType::velocity` for balance;
- vendor-aligned velocity PI / LPF values;
- `motor.loopFOC()` and `motor.move(...)`.

## Local build

```bash
git submodule update --init --recursive
cd tools/simplefoc_compat
idf.py set-target esp32
idf.py build
```

Do **not** flash this compatibility application. Its `app_main()` deliberately references initialization and motor APIs so the linker cannot optimize the compatibility surface away.

## Gate after a successful build

A green build only permits the next software step: introduce a controlled SimpleFOC owner/adapter into the real runtime and add parity tests. Hardware authority remains blocked until the following are re-validated against the vendor baseline:

- AS5600 direction and wheel-speed sign;
- electrical alignment / pole pairs;
- 3-PWM phase order and polarity through EG2133;
- velocity PI and LPF semantics;
- torque/voltage swing mode;
- velocity balance mode;
- mode-transfer reset / bumpless behavior.
