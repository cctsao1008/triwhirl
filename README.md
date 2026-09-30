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
  <em>Measure first. Model carefully. Control deliberately.</em>
</p>

<p align="center">
  🔺 Geometry &nbsp;·&nbsp; 🌀 Momentum &nbsp;·&nbsp; 🧠 Fuzzy Control &nbsp;·&nbsp; 🔬 Validate
</p>

TriWhirl is an ESP32 research platform for a reaction-wheel-stabilized Reuleaux triangle. The project separates physical evidence, state estimation, full-fuzzy attitude control, motor control, safety authority, transport, simulation, and post-run analysis so each layer can be validated independently.

> **Curved triangle. Hard evidence. No host-assisted balance.**

---

## 🧠 Target architecture

The research direction is now explicit:

- **all system-level attitude control is fuzzy**;
- **SimpleFOC owns the reaction-wheel motor-control layer**;
- **AS5600 belongs to SimpleFOC**;
- the fuzzy controller consumes body state plus reaction-wheel velocity and emits only a wheel `target_velocity`;
- motor velocity regulation, FOC, and PWM remain SimpleFOC responsibilities;
- deterministic safety always overrides fuzzy intent.

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

The fuzzy attitude layer owns swing-up, capture, near-upright balance, disturbance recovery, and reaction-wheel momentum management. It does **not** emit direct `Vq`, phase voltage, duty cycle, or PWM.

## 🧩 Control boundary

The high-level control problem is:

```text
(theta, theta_dot, wheel_velocity, derived physical state)
                         |
                         v
                full-fuzzy inference
                         |
                         v
              target_velocity [rad/s]
                         |
                         v
                 SimpleFOC velocity
                         |
                         v
                    motor torque
```

Swing-up, capture, and balance are treated as regions of one nonlinear attitude-control problem rather than preserving a legacy voltage-mode/velocity-mode split.

The first implementation may use separate rule groups for engineering clarity, but the composed actuator request remains a continuous wheel-velocity target.

## 🌀 Reaction-wheel momentum

Wheel velocity is a control state, not merely telemetry.

A controller that keeps the body visually upright while allowing wheel momentum to drift toward saturation is not considered stable enough for TriWhirl. Fuzzy rules and validation therefore include explicit momentum unloading / authority management.

## ⚙️ Motor layer

TriWhirl targets the **latest stable SimpleFOC release**, pinned to an explicit release/tag rather than tracking upstream `master`.

As of 2026-09-30, the latest stable release is **SimpleFOC v2.4.0**.

SimpleFOC owns:

```text
AS5600 sensor integration
shaft angle / shaftVelocity()
velocity closed loop
motor torque/current/voltage implementation
FOC electrical-angle handling
PWM generation
```

`triwhirl_core` should remain independent of SimpleFOC APIs. A narrow hardware/motor adapter exposes only the wheel state and bounded target-velocity interface required by the attitude controller.

## 🚫 Vendor-source independence

The supplied vendor control program is archival material only. It is not a design, tuning, parity, or validation authority.

TriWhirl does not use vendor source code as the source of truth for:

- control-law structure or gains;
- swing-up behavior;
- capture thresholds;
- SimpleFOC PI/LPF tuning;
- sensor sign convention;
- motor limits;
- expected dynamic behavior;
- golden traces or pass/fail criteria.

Hardware facts must be supported by schematic evidence, component datasheets, mechanical characterization, or our own bench measurements. Unknown values remain explicit unknowns until measured.

## 🛡️ Safety authority

Safety remains deterministic and outside fuzzy inference.

```text
fuzzy controller intent
        |
        v
hard limits / validity / fault policy
        |
        v
authorized target_velocity
        |
        v
SimpleFOC motor layer
```

Examples include:

- stale or invalid sensor data;
- hard wheel-speed limits;
- numerical sanity checks;
- motor / FOC faults;
- emergency disable;
- unrecoverable-fall handling.

UART, BLE, Python, plotting, and browser/host state never own realtime motor authority.

## 🧰 Firmware shape

```text
triwhirl/
├── main/                       application wiring / runtime orchestration
├── components/
│   ├── triwhirl_core/          geometry, state, fuzzy control, safety math
│   ├── triwhirl_hw/            ESP32 board / sensor / motor integration
│   └── triwhirl_ble/           ESP-IDF NimBLE transport
├── tools/                      commissioning, logging, ID, fitting, SITL
├── docs/                       durable technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
└── README.md
```

Platform-independent fuzzy-control logic belongs in `triwhirl_core`. SimpleFOC integration belongs behind the hardware/motor boundary rather than leaking through control-law code.

## 🧪 SITL

Simulation uses the same command semantics as firmware:

```text
body / wheel state
       |
       v
full-fuzzy attitude controller
       |
       v
target_velocity
       |
       v
identified motor velocity-servo model
       |
       v
reaction-wheel / Reuleaux plant
```

A velocity target must never be injected into the plant as if it were `Vq`.

The near-upright local model remains provisional evidence. Full-swing simulation remains observational until geometry, COM, inertia, contact, and rolling-loss parameters are supported well enough to justify stronger validation authority.

## 🔬 Validation

TriWhirl treats captured data as evidence, not decoration.

Validation is against explicit physical/control requirements rather than matching vendor behavior. Important metrics include:

```text
peak / RMS body-angle error
capture and settling time
peak / RMS body rate
peak / RMS wheel velocity
wheel-speed saturation dwell
target-velocity continuity
body-disturbance recovery
wheel-disturbance recovery
momentum unloading effectiveness
robustness across plant uncertainty
long-duration upright stability
```

Captured timing, framing, sequence continuity, transport loss, firmware provenance, and measured sample intervals remain part of the evidence chain.

## 📏 Physical parameter gate

Physical values are admitted only from supported evidence:

```text
schematic / datasheet / measurement
              |
              v
      identified evidence
              |
              v
       admissible parameter
              |
              v
      controller / safety use
```

Examples that require independent establishment include motor pole pairs, phase order, encoder direction, safe wheel-speed range, supply behavior, motor electrical limits, COM location, inertia, and rolling loss.

## 🧰 Host toolbox

`twtool` remains the canonical host entry point for logging, inspection, experiments, fitting, and evidence handling:

```powershell
python tools/twtool.py --help
python tools/twtool.py --list
```

Host tools support engineering work but never close the realtime balance loop.

## 🔧 Build and motor-layer migration

The current repository is migrating from the existing native motor path to the latest-stable SimpleFOC ownership model tracked by Issue #39.

The compatibility work currently probes:

```text
ESP-IDF 6.1
+ Arduino-ESP32 integration
+ SimpleFOC v2.4.0
```

Toolchain compatibility is not hardware validation. If the current Arduino/ESP-IDF combination is incompatible, the integration stack may change while SimpleFOC remains on the latest stable release target.

No standup hardware trial is authorized merely because the compatibility project builds.

## 📚 Documentation

- [Full-fuzzy control boundary](docs/full-fuzzy-control.md)
- [Architecture](docs/architecture.md)
- [Hardware](docs/hardware.md)
- [Development](docs/development.md)
- [Motor bring-up](docs/motor-bringup.md)
- [Host tools](tools/README.md)
- [Parameter identification](tools/parameter_id/README.md)

## Documentation principle

> **README explains the system. Issues explain the journey. Code proves the current state.**

Durable documentation describes architecture, control boundaries, runtime authority, hardware semantics, and validation interpretation. GitHub Issues preserve experiments, temporary constraints, migration work, and closure records. Code, configuration, captured evidence, and tests remain the authoritative proof of executable behavior.

## License

MIT
