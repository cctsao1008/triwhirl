# SimpleFOC compatibility spike

Issues: #39, #43

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

A green build proves API/toolchain compatibility only. It does not prove motor behavior, tuning, scheduling, or hardware readiness.

SimpleFOC v2.4.0 explicitly documents Arduino-ESP32 3.x compatibility. Therefore Arduino-ESP32 4.x / ESP-IDF 6.1 remains an intentional compatibility experiment. We will not downgrade SimpleFOC to recover compatibility.

## ESP32 PWM backend on IDF 6.1

The first Route-B compile reached SimpleFOC itself and exposed a real v2.4.0 / IDF-6 incompatibility in the default ESP32 MCPWM current-sense backend. SimpleFOC v2.4.0 carries a copy of ESP-IDF 5.1.4 private MCPWM structures; ESP-IDF 6.1 changed that private ABI and replaced the old `SOC_MCPWM_*_PER_GROUP` resource-count macros with the new MCPWM LL interface.

TriWhirl will not copy or locally redefine Espressif private MCPWM structures just to make the build pass. The Route-B integration therefore selects SimpleFOC's own supported ESP32 LEDC backend with:

```text
SIMPLEFOC_ESP32_USELEDC
```

This keeps the library unmodified and keeps motor velocity control, FOC and PWM under SimpleFOC ownership. The MCPWM backend may be revisited when SimpleFOC gains upstream ESP-IDF-6 support or when a separately reviewed compatibility layer is justified.

This decision does **not** establish that LEDC is already commissioned for the hardware. PWM frequency, duty behavior and motor-loop performance remain measurement gates before balancing trials.

## MotorControl boundary

The probe now reaches SimpleFOC through the shared, library-independent `triwhirl::MotorControl` boundary rather than allowing the system-level controller to access `BLDCMotor` directly.

The attitude-facing contract is deliberately mechanical only:

```text
attitude/control domain                 motor/SimpleFOC domain
-----------------------                 ----------------------
command target_velocity_rad_s   ----->  consume latest target
read cached shaft_velocity      <-----  publish latest snapshot
                                        loopFOC / move / AS5600
```

`MotorControl` exposes bounded target velocity, stop, and a cached shaft-velocity/validity snapshot. It does **not** expose electrical angle, phase voltages, PWM state, `Vq`, or SimpleFOC types.

The probe backend separates command/snapshot operations from `serviceBackend()`. The former only touch cached scalar state. The latter is where SimpleFOC calls and AS5600 access occur. This separation is intentional: the production attitude loop must not acquire a synchronous I2C dependency simply because SimpleFOC owns the encoder.

`stop()` is a non-blocking **disable request**, not a zero-speed hold. The probe motor domain applies that request with `motor.disable()` on its next service cycle; a later velocity command re-enables the motor in that same backend domain. This gives the interface the correct de-energized stop semantics without making the attitude task call SimpleFOC directly.

That asynchronous stop path is **not yet a hard-safety guarantee**. Production work must bound motor-service latency and decide whether safety faults also require a lower-level driver-enable or hardware interlock that is independent of the normal command mailbox.

The probe backend itself is not presented as a cross-core synchronization implementation. The exact production motor-task core, service frequency, coherent command/snapshot primitive, scheduling policy, stop latency, and hard-safety path are not established by this compile probe.

## Dependency policy

The probe intentionally does **not** vendor SimpleFOC as a Git submodule. PlatformIO resolves the pinned SimpleFOC commit through `lib_deps`.

The `platformio.ini` pins:

- pioarduino `platform-espressif32` IDF-6 preparation commit;
- Arduino-ESP32 4.0.0-RC1 exact commit;
- SimpleFOC v2.4.0 exact release commit.

"Track latest" means explicitly reviewing and updating these pins when a newer Arduino-ESP32 4.x RC/stable or newer stable SimpleFOC release is selected. It does not mean floating `master`/`develop` dependencies in CI.

## APIs exercised

Inside the SimpleFOC backend, the probe still compiles and links the architecture-relevant library path:

- `MagneticSensorI2C(AS5600_I2C)`;
- `motor.linkSensor(&sensor)`;
- `motor.shaftVelocity()`;
- `BLDCMotor(...)`;
- `BLDCDriver3PWM(...)`;
- `MotionControlType::velocity`;
- `motor.init()` / `motor.initFOC()` / `motor.enable()` / `motor.disable()` / `motor.loopFOC()`;
- `motor.move(target_velocity)`.

The probe application itself interacts through `MotorControl::commandTargetVelocityRadS()`, `MotorControl::observation()`, `MotorControl::stop()`, and the motor-domain `MotorControl::serviceBackend()` hook.

Any numeric pin, bus, pole-pair, voltage, or velocity values in this compile-only target are placeholders unless independently supported by schematic/datasheet/measurement evidence.

No vendor PI/LPF gains, swing behavior, capture logic, sign convention, or motor limits belong in this probe.

## Local build

Install a pioarduino-compatible PlatformIO core, then run:

```bash
pio run -d tools/simplefoc_compat
```

Do **not** flash this compatibility application. It exists only to compile and link the selected API surface.

## Gate after a successful adapter build

A green adapter build permits the next software step only: design the production motor execution domain and migrate runtime ownership without putting synchronous SimpleFOC/AS5600 work into the attitude task.

Before any balancing hardware trial, independently characterize:

- AS5600 direction and wheel-speed sign;
- motor pole pairs;
- 3-PWM phase order and driver polarity;
- electrical alignment behavior;
- safe supply/motor voltage and current limits;
- safe wheel-speed envelope;
- SimpleFOC velocity-loop step response, bandwidth, overshoot, and saturation;
- velocity-estimator noise and latency;
- selected PWM-backend frequency/resolution and duty-update behavior;
- motor-domain execution time, jitter, and interaction with the 1 kHz attitude loop;
- commanded-stop to de-energized latency and the independent hard-safety shutdown path.

The full-fuzzy attitude controller will consume the published shaft-velocity snapshot and emit only bounded `target_velocity` in rad/s. The motor velocity servo, FOC, PWM, and AS5600 path remain SimpleFOC responsibilities.
