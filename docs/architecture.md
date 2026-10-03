# Architecture

TriWhirl separates **current live firmware**, **target control architecture**, and **engineering/offline tooling** so migration work does not silently redefine runtime authority.

## Top-level layout

```text
triwhirl/
├── main/                       runtime wiring / current live firmware
├── components/
│   ├── triwhirl_core/          platform-independent geometry/control/safety math
│   ├── triwhirl_hw/            native ESP-IDF board/peripheral integration
│   ├── triwhirl_simplefoc/     production-shaped Route-B SimpleFOC backend
│   └── triwhirl_ble/           native ESP-IDF NimBLE transport
├── tools/                      logging, identification, fitting, SITL, offline tools
├── docs/                       durable technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── LICENSE
```

## Component boundaries

`main/` owns application wiring and runtime orchestration.

`components/triwhirl_core/` contains project-owned deterministic geometry, estimation, fuzzy inference/control foundations, safety math, and signal-processing code. It must remain independent of Arduino, PlatformIO, SimpleFOC, and ESP32 peripheral APIs.

`components/triwhirl_hw/` contains the current native ESP-IDF board/peripheral integration.

`components/triwhirl_simplefoc/` contains the production-shaped target motor backend used by the pinned Route-B Arduino/SimpleFOC build. It is compiled/linked as part of the migration proof but is not yet the live `app_main()` motor owner.

`components/triwhirl_ble/` contains the native ESP-IDF NimBLE GATT transport. Transport is engineering/debug I/O and does not own control state.

`tools/` contains host/browser engineering utilities for logging, identification, modeling, historical controller synthesis, and simulation. Host tools do not own realtime motor authority.

## Current live runtime

The live firmware still uses the legacy native motor path while migration gates are completed:

```text
AS5600 + MPU6050
       |
       v
native sensing / state estimation
       |
       v
safety / supervisor authority
       |
       v
legacy commissioning controller -> Vq
       |
       v
native motor realization / MCPWM -> BLDC
```

This path is commissioning and rollback scaffolding. It is not the target research architecture and the historical/vendor controller is not a design or tuning authority.

The deterministic control opportunity remains 1 kHz. Communication/UI paths remain outside realtime authority:

```text
                 ┌─ CH340 / UART
runtime protocol ┤
                 └─ NimBLE GATT -> host tools / WebUI
```

Transport disconnects do not grant, own, or reset motor authority.

## Target control architecture

The target system boundary is:

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
          full-fuzzy attitude control
                   |
                   | target_velocity [rad/s]
                   v
              MotorControl boundary
                   |
                   v
          SimpleFOC velocity / FOC / PWM
                   |
                   v
                  BLDC
```

The fuzzy system controller owns swing-up, capture, balance, disturbance recovery, and reaction-wheel momentum management. It emits only bounded `target_velocity`.

SimpleFOC owns the target-path AS5600 integration, wheel velocity regulation, electrical angle, FOC, and PWM. No target-path fuzzy-to-`Vq` interface exists.

The software execution-domain proof already separates the attitude/control domain from motor work through coherent latest-value command and observation mailboxes. A target-side motor task/scheduling wrapper and production-shaped SimpleFOC backend compile/link in the pinned Route-B graph without activation in `app_main()`.

See [`full-fuzzy-control.md`](full-fuzzy-control.md) for the detailed control contract.

## Safety authority

Safety is deterministic, project-owned, and independent of fuzzy inference, UART, BLE, or browser state.

Examples include sensor-validity/staleness policy, non-finite numerical checks, hard wheel/command limits, backend faults, initialization/alignment failures, emergency disable, and unrecoverable-fall policy.

The current native board path and the future SimpleFOC path may have different low-level electrical semantics, but both remain subordinate to the same project-level safety authority.

## Simulation boundary

SITL mirrors the target mechanical-domain command semantics:

```text
state -> fuzzy target_velocity [rad/s]
      -> motor velocity-servo model
      -> wheel_accel_command [rad/s^2]
      -> mechanical plant
```

The historical provisional local B/C model was identified in a Vq coordinate. A simulation-only algebraic transformation now preserves that local model when driven by commanded wheel acceleration. Its latent `Vq_equiv` is not a production command path.

The local B/C models remain provisional near-upright evidence. The global geometry-derived swing model remains exploratory with `validation_authority=NONE`.

## Dependency rules

- `triwhirl_core` does not depend on Arduino, PlatformIO, SimpleFOC, or ESP32 peripheral APIs.
- hardware/backend layers may depend on their selected platform APIs.
- the attitude controller communicates with the target motor layer through mechanical-domain types, not SimpleFOC classes.
- the ESP32 runtime does not depend on Python or an online solver.
- legacy Vq and robust-control tools may remain as historical/engineering artifacts, but they do not define the active target architecture.

## Migration rule

Software compile/link success, deterministic host tests, or SITL success do not by themselves authorize hardware balancing. Runtime ownership moves only after the relevant execution-domain, motor-commissioning, model, and safety gates are supported by independent evidence.
