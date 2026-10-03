<p align="center">
  <img src="docs/assets/triwhirl-mascot.svg" width="240" alt="TriWhirl mascot">
</p>

<h1 align="center">TriWhirl</h1>

<p align="center">
  <strong>Reaction-Wheel Reuleaux Triangle Control Research</strong>
</p>

<p align="center">
  <strong>Three vertices. One reaction wheel. Zero imaginary physics.</strong>
</p>

<p align="center">
  <em>Measure first. Separate evidence from authority. Migrate deliberately.</em>
</p>

TriWhirl is an embedded control and validation platform for a reaction-wheel-stabilized Reuleaux triangle. The repository is currently in a controlled migration: the live/default firmware still contains a native ESP-IDF Vq/MCPWM commissioning path, while the approved target architecture is full-fuzzy attitude control over a SimpleFOC velocity-controlled reaction-wheel layer.

> **Current runtime is not the same thing as target architecture.**

---

## 🧠 Architecture status

### Current executable runtime

The current hardware runtime remains the legacy commissioning path:

```text
AS5600 + MPU6050
       ↓
state estimation
       ↓
deterministic safety / supervisor
       ↓
legacy commissioning controller
       ↓
bounded Vq
       ↓
native MCPWM
       ↓
EG2133 -> BLDC -> reaction wheel
```

This path remains valuable as rollback scaffolding, board-commissioning infrastructure, and regression evidence. It is **not** the active research target.

### Target architecture

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

The fuzzy system layer owns swing-up, capture, near-upright balance, disturbance recovery, and reaction-wheel momentum management. It emits **only** bounded wheel `target_velocity` in rad/s.

SimpleFOC owns the target-path AS5600 integration, shaft velocity, motor velocity servo, electrical angle, FOC, and PWM. No target-path fuzzy rule emits `Vq`, phase voltage, duty cycle, or PWM directly.

Deterministic safety remains outside fuzzy inference and always overrides controller output.

## 🧱 Migration boundary already proven

The repository already contains the software contracts needed to keep the system and motor domains separate:

```text
attitude/control domain                 motor/SimpleFOC domain
-----------------------                 ----------------------
read shaft snapshot            <-----   publish shaftVelocity
write target_velocity_rad_s    ----->   consume latest target
                                       AS5600 / loopFOC / move
```

The cross-domain transport is bounded latest-value SPSC mailboxes rather than shared mutable floats. The motor execution-domain and scheduling contracts are host-tested and target-compiled, while live AS5600/PWM ownership remains intentionally unmigrated until hardware commissioning and timing gates are satisfied.

The production-shaped SimpleFOC backend is present but inert in the default native build unless the Route-B integration path is explicitly selected.

## 🧠 Fuzzy-control boundary

TriWhirl owns its embedded fuzzy inference implementation:

- symmetric five-term normalized memberships: `NL / NS / ZE / PS / PL`;
- allocation-free zero-order Sugeno inference;
- 1-D, 2-D, and 3-D rule surfaces;
- deterministic finite-value and bounded-input semantics.

FuzzyLite is pinned under `third_party/` only as a **host/reference oracle**. It is not a production firmware dependency.

```text
reference controller definition
            │
      ┌─────┴─────┐
      ▼           ▼
  FuzzyLite    TriWhirl fuzzy core
  host only    embedded/project-owned
      └──── parity / regression ────┘
```

Inference parity is not stability proof. Closed-loop fuzzy rule tuning is validated separately against explicit local SITL and later hardware evidence.

## 🧪 SITL evidence boundary

Near-upright simulation now follows the target command abstraction:

```text
body / wheel state
       ↓
attitude controller
       ↓
target_velocity
       ↓
VelocityServoModel
       ↓
wheel_accel_command
       ↓
provisional local B/C plant
```

The wheel-acceleration coordinate is a local modeling transform around the same provisional near-upright plant. It is **not** a claim that the real motor is an ideal acceleration source.

Global full-swing simulation remains observational with:

```text
validation_authority=NONE
```

until global geometry, COM, inertia, rolling loss, and contact behavior are sufficiently supported.

## 🛡️ Safety authority

Safety is deterministic and independent of UART, BLE, WebUI, fuzzy rule firing, or host state.

Typical safety responsibilities include:

- sensor validity and freshness;
- non-finite state rejection;
- hard command and wheel-speed limits;
- backend/motor fault propagation;
- timeout handling;
- fail-closed stop behavior.

The current native board-safe stop uses the EG2133 low-side zero vector; it is not described as high impedance.

## 📦 Repository shape

```text
triwhirl/
├── main/                       runtime composition / current firmware wiring
├── components/
│   ├── triwhirl_core/          estimation, fuzzy/control math, safety, contracts
│   ├── triwhirl_hw/            current native ESP32 hardware path
│   ├── triwhirl_simplefoc/     production-shaped SimpleFOC backend
│   └── triwhirl_ble/           NimBLE engineering transport
├── controllers/                fuzzy reference/controller definitions
├── third_party/                host-only pinned reference dependencies
├── tools/                      validation, logging, ID, SITL, synthesis utilities
├── docs/                       durable technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
└── README.md
```

`triwhirl_core` remains platform-independent. It does not depend on Arduino, ESP-IDF peripheral APIs, FuzzyLite, or Python.

## 🔬 Evidence policy

TriWhirl treats measurements, captures, tests, and model assumptions as different kinds of evidence. A number does not become a physical fact merely because it appears in seller code, a temporary tuning run, or a convenient simulation.

```text
schematic / datasheet / measurement
              ↓
         supported fact
              ↓
     controller or safety use
```

Unknown physical quantities remain explicit unknowns until supported. Important examples include final safe wheel-speed limits, bus behavior under load, global COM/inertia parameters, rolling loss, and final motor-servo dynamics.

## 📜 Vendor-source policy

The supplied vendor program is archival only. It is **not** authority for:

- fuzzy rules or controller gains;
- SimpleFOC PI/LPF values;
- sign conventions;
- motor limits;
- swing/capture behavior;
- expected dynamics;
- golden traces;
- validation thresholds.

Hardware facts come from schematics, component datasheets, geometry, or independent bench measurement.

## 🗃️ Legacy Vq / H∞ tooling

The repository still contains Vq-based identification, H∞ synthesis, and legacy stand-up tooling. These artifacts are retained because they contain useful commissioning, modeling, and historical evidence.

They are **not** the active target control architecture. New system-control work converges toward full-fuzzy attitude control producing `target_velocity` for SimpleFOC velocity mode.

## 🧰 Host toolbox

`twtool` remains the canonical host entry point for current commissioning and evidence workflows:

```powershell
python tools/twtool.py --help
python tools/twtool.py --list
```

Current command groups include logging, diagnostics, control/commissioning, identification, plant tooling, and legacy synthesis utilities. Host tools remain supervisory; the ESP32 is the realtime authority.

See [`tools/README.md`](tools/README.md) for the current command surface.

## 🔧 Build

The default firmware remains the native ESP-IDF build:

```text
ESP-IDF      v6.1
target       ESP-WROOM-32 / classic ESP32
runtime      native ESP-IDF C/C++ + CMake
```

Build:

```bash
idf.py build
```

A separate pinned Route-B CI path proves coexistence with Arduino-ESP32 4.x and SimpleFOC v2.4.0 without making that dependency part of the default native firmware build.

## 📚 Documentation

- [Architecture](docs/architecture.md)
- [Hardware](docs/hardware.md)
- [Development](docs/development.md)
- [Motor bring-up](docs/motor-bringup.md)
- [Host tools](tools/README.md)
- [Parameter identification](tools/parameter_id/README.md)

## Documentation principle

> **README explains the system. Issues explain the journey. Code and evidence prove the current state.**

Durable documentation distinguishes current implementation, target architecture, simulation assumptions, and hardware evidence so that one layer cannot silently borrow authority from another.

## License

MIT
