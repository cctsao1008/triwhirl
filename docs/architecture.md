# Architecture

TriWhirl separates the **currently executable legacy runtime** from the **approved target control architecture**. The repository is mid-migration; documentation must not confuse compile/link or SITL evidence with ownership already transferred on hardware.

## Top-level layout

```text
triwhirl/
├── main/                       application wiring / entry point
├── components/
│   ├── triwhirl_core/          platform-independent estimation/control/safety math
│   ├── triwhirl_hw/            current native ESP32 peripheral integration
│   ├── triwhirl_simplefoc/     production-shaped SimpleFOC motor backend
│   └── triwhirl_ble/           native ESP-IDF NimBLE transport
├── controllers/                controller definitions / reference artifacts
├── third_party/                host-only reference dependencies
├── tools/                      logging, validation, identification, simulation tools
├── docs/                       durable technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── LICENSE
```

## Component boundaries

`main/` owns application wiring and runtime orchestration.

`components/triwhirl_core/` contains project-owned deterministic estimation, safety, control, fuzzy inference, and signal-processing code that does not depend on ESP32 peripheral APIs.

`components/triwhirl_hw/` contains the **current legacy native** ESP32 board/peripheral path. During migration it remains rollback and commissioning scaffolding; it is not the target motor-control architecture.

`components/triwhirl_simplefoc/` contains the production-shaped SimpleFOC backend proven in the Route-B build graph. Its existence and compile/link coverage do not by themselves transfer live AS5600/PWM ownership.

`components/triwhirl_ble/` contains the native ESP-IDF NimBLE GATT transport. It is an engineering/debug transport and does not own control state.

`tools/` contains PC/browser-side engineering utilities for logging, validation, parameter identification, modeling, legacy H-infinity work, and simulation. Host tools do not own realtime actuation authority.

## Current executable runtime

The current live/default runtime still uses the legacy native motor owner:

```text
AS5600 + MPU6050
       ↓
state estimation
       ↓
deterministic safety / supervisor
       ↓
legacy commissioning control
       ↓
bounded Vq
       ↓
native MCPWM -> EG2133 -> BLDC
```

This path remains useful for rollback, board commissioning, regression evidence, and migration comparison. It is **not** the active research target and must not be treated as authority for the new fuzzy/SimpleFOC control law.

## Target architecture

The approved target architecture is:

```text
MPU6050                         AS5600
   │                               │
   │ theta, theta_dot              ▼
   │                           SimpleFOC
   │                        shaftVelocity()
   │                               │
   └──────────────┬────────────────┘
                  ▼
        full-fuzzy attitude control
                  │
                  │ bounded target_velocity [rad/s]
                  ▼
       MotorControl / execution domain
                  │
                  ▼
        SimpleFOC velocity control
                  │
             FOC / PWM
                  │
                  ▼
            EG2133 -> BLDC
```

The full-fuzzy attitude layer owns swing-up, capture, near-upright balance, disturbance recovery, and wheel-momentum management. It emits only a bounded wheel `target_velocity` in rad/s.

SimpleFOC owns the target-path AS5600 integration, shaft velocity, motor velocity loop, electrical angle, FOC, and PWM. The fuzzy layer does **not** emit `Vq`, phase voltage, duty cycle, or PWM.

The target command/state crossing uses the proven latest-value command/observation mailbox and motor execution-domain contracts. Runtime ownership handoff is still gated by independent hardware commissioning and timing evidence.

## Deterministic safety boundary

Safety remains outside fuzzy inference. Sensor validity, stale-data handling, numerical sanity, hard wheel/command limits, backend faults, emergency disable, and other fail-closed guards override controller output.

The current board-safe stop is the EG2133 low-side zero vector; it is not described as high impedance. If target-path stop latency or de-energization requirements demand an independent hardware inhibit, that decision belongs to the safety/motor execution layer rather than fuzzy rules.

## Simulation evidence boundary

Near-upright SITL currently has two explicitly separated coordinates:

```text
target_velocity
      ↓
VelocityServoModel
      ↓
wheel_accel_command
      ↓
provisional local B/C plant
```

The local wheel-acceleration coordinate is an algebraic/modeling bridge around the same provisional near-upright plant. It does not claim that the real motor is an ideal acceleration source.

Global full-swing simulation remains observational with `validation_authority=NONE` until geometry, COM, inertia, rolling loss, and related global plant parameters earn stronger evidence.

## Fuzzy reference boundary

TriWhirl owns the embedded deterministic fuzzy runtime. FuzzyLite is pinned as a **host-only reference/validation oracle** and is not part of the production firmware dependency graph.

```text
.fll / reference controller
          │
    ┌─────┴─────┐
    ▼           ▼
FuzzyLite    TriWhirl fuzzy core
host only    deterministic embedded code
    └──── parity / regression ────┘
```

The generic fuzzy core provides allocation-free five-term membership plus zero-order Sugeno 1-D/2-D/3-D inference. Controller tuning is a separate step; parity/contract tests validate inference semantics, not closed-loop stability.

## Vendor-source policy

The supplied vendor program is archival only. It is not authority for gains, fuzzy rules, SimpleFOC PI/LPF values, sign conventions, limits, swing/capture behavior, golden traces, or acceptance criteria.

Hardware facts come from schematics, component datasheets, geometry, or independent measurement. Unknown values remain unknown until supported.

## Legacy model/synthesis tooling

The repository may retain Vq-based plant-identification and H-infinity tooling because it contains useful historical, commissioning, and modeling evidence. Those tools are not the active target controller architecture and must not be described as the path that production full-fuzzy control is converging toward.

## Dependency rule

Application wiring may depend on project components. Hardware-specific code may depend on ESP-IDF/Arduino/SimpleFOC only inside its designated motor/hardware boundary. `triwhirl_core` remains platform-independent.

The ESP32 runtime does not depend on Python, FuzzyLite, or an online convex solver. Offline tools produce evidence or reference artifacts consumed by deterministic project code.
