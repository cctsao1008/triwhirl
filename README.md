<p align="center">
  <img src="docs/assets/triwhirl-mascot.svg" width="240" alt="TriWhirl mascot">
</p>

<h1 align="center">TriWhirl</h1>

<p align="center">
  <strong>Reaction-Wheel Reuleaux Triangle Control Research</strong>
</p>

<p align="center">
  <em>Measure first. Model carefully. Control deliberately.</em>
</p>

<p align="center">
  🔺 Geometry &nbsp;·&nbsp; 🌀 Momentum &nbsp;·&nbsp; 🧠 Fuzzy Control &nbsp;·&nbsp; 🔬 Validate
</p>

TriWhirl is an ESP32 research platform for a reaction-wheel-stabilized Reuleaux triangle. The project separates physical evidence, state estimation, system-level attitude control, motor control, deterministic safety, runtime authority, transport, simulation, and post-run analysis so each layer can be validated without claiming more certainty than the evidence supports.

> **Curved triangle. Hard evidence. No host-assisted balance.**

---

## Target architecture

The approved research direction is **full-fuzzy attitude control over a latest-stable SimpleFOC motor layer**:

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

The fuzzy attitude layer owns swing-up, capture, near-upright balance, disturbance recovery, and reaction-wheel momentum management. Its system-level actuator output is always a bounded wheel `target_velocity` in rad/s.

The fuzzy controller does **not** emit direct `Vq`, phase voltage, duty cycle, or PWM. SimpleFOC owns the wheel velocity loop, electrical angle, FOC, and PWM in the target architecture.

See [`docs/full-fuzzy-control.md`](docs/full-fuzzy-control.md) for the durable control-boundary contract.

## Current runtime versus target runtime

The repository is intentionally in a staged migration.

### Current live firmware

The current live ESP-IDF runtime still contains the older native motor path:

```text
AS5600 -> WheelKinematics
                |
MPU6050 -> attitude/state
                |
         legacy commissioning control
                |
               Vq
                |
         native FOC / MCPWM
                |
             BLDC
```

That path remains useful as commissioning/rollback scaffolding while the replacement motor execution domain is proven. Its existence is **not** the target control architecture and does not make the old controller a design authority.

### Target motor execution domain

The replacement path is built around a library-independent mechanical boundary:

```text
attitude/control domain                 motor/SimpleFOC domain
-----------------------                 ----------------------
read shaft snapshot            <-----   publish shaftVelocity
write target_velocity_rad_s    ----->   consume latest target
                                       AS5600 / loopFOC / move
```

The repository already proves coherent command/snapshot transport, fail-closed motor-execution semantics, target-side task/scheduling structure, and a production-shaped SimpleFOC backend that compiles/links in the pinned Route-B graph without being activated by `app_main()`.

Those software gates do not authorize hardware balancing or claim measured physical command-to-PWM timing.

## Control boundary

The high-level control problem is:

```text
(theta_error, theta_dot, wheel_velocity, justified derived state)
                            |
                            v
                   full-fuzzy inference
                            |
                            v
                 target_velocity [rad/s]
                            |
                            v
                     SimpleFOC velocity
```

The initial fuzzy state is body angle error, body angular rate, and reaction-wheel velocity. Wheel velocity is a control state because an apparently upright body with silently accumulating reaction-wheel momentum is not a satisfactory stable solution.

Safety is deterministic and outside fuzzy inference. Sensor validity, stale-data policy, non-finite numerics, hard wheel/command limits, motor/backend faults, emergency disable, and unrecoverable-fall handling override fuzzy intent.

## Vendor-source independence

The supplied vendor program is archival material only. TriWhirl does not use it as the source of truth for:

- LQR/state-feedback structure or gains;
- swing-up or capture logic;
- SimpleFOC PI/LPF tuning;
- sensor sign conventions;
- motor limits;
- expected dynamics;
- golden traces or pass/fail behavior.

Hardware facts come from schematics, component datasheets, mechanical characterization, or independent measurement. Unknown values remain explicit unknowns until measured.

## Motor layer

The target motor layer uses an explicitly pinned latest-stable SimpleFOC release rather than floating upstream `master`. The current reviewed integration line is SimpleFOC v2.4.0 with the pinned Route-B Arduino-ESP32 / ESP-IDF 6.1 stack.

`triwhirl_core` remains independent of Arduino and SimpleFOC APIs. SimpleFOC types stay behind the motor/backend boundary.

The current Route-B integration selects SimpleFOC's LEDC PWM backend explicitly because the default v2.4.0 ESP32 MCPWM implementation depends on older ESP-IDF private ABI details. TriWhirl does not copy or redefine Espressif private MCPWM structures merely to force compatibility.

Motor commissioning values such as phase order, pole pairs, sensor direction, electrical alignment, safe voltage/current limits, velocity-loop tuning, speed envelope, and PWM behavior require independent evidence before runtime ownership can move.

## SITL and model authority

Simulation uses the same command semantics as the target architecture:

```text
body / wheel state
       |
       v
full-fuzzy attitude controller
       |
       v
target_velocity [rad/s]
       |
       v
motor velocity-servo model
       |
       v
wheel_accel_command [rad/s^2]
       |
       v
reaction-wheel / Reuleaux plant
```

A velocity target is never injected into a Vq-input plant as though the units were interchangeable.

The existing near-upright B/C models remain **provisional local evidence**. TriWhirl now has deterministic contracts for the target-velocity servo boundary and for the algebraic transformation from the historical local Vq coordinate to commanded wheel acceleration. The latent `Vq_equiv` used by that transform is simulation-only; it is not a production command path.

The geometry-derived full-swing model remains exploratory and explicitly reports:

```text
validation_authority=NONE
```

A simulated full swing-up result is therefore observation, not validation authority.

## Fuzzy inference foundation

`components/triwhirl_core/include/triwhirl/fuzzy.hpp` provides a small allocation-free inference foundation:

```text
five linguistic terms: NL / NS / ZE / PS / PL
normalized universe:    [-1, +1]
inference:              zero-order Takagi-Sugeno
supported dimensions:   1-D / 2-D / 3-D
```

Normalization, rule surfaces, physical output scaling, momentum policy, and limits belong to the actual attitude controller rather than the generic inference primitive.

The generic inference core has deterministic host-side CI for partition, symmetry, shoulder/clamping behavior, non-finite rejection, 1-D/2-D/3-D Sugeno behavior, and membership-boundary continuity.

## Firmware shape

```text
triwhirl/
├── main/                       runtime orchestration / current live firmware
├── components/
│   ├── triwhirl_core/          platform-independent geometry/control/safety math
│   ├── triwhirl_hw/            native ESP-IDF hardware integration
│   ├── triwhirl_simplefoc/     production-shaped Route-B SimpleFOC backend
│   └── triwhirl_ble/           native ESP-IDF NimBLE transport
├── tools/                      commissioning, logging, ID, fitting, SITL
├── docs/                       durable technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
└── README.md
```

Engineering transports remain outside realtime motor authority:

```text
                 ┌─ CH340 / UART
runtime protocol ┤
                 └─ NimBLE GATT -> host toolbox
```

UART, BLE, Python, plotting, and browser state do not grant motor authority.

## Evidence and commissioning

TriWhirl treats captured hardware data as evidence, not decoration. Firmware records synchronized state/timing; host-side tooling decodes, inspects, plots, and fits after acquisition rather than putting host I/O inside the realtime control loop.

`twtool` is the canonical host entry point:

```powershell
python tools/twtool.py --help
python tools/twtool.py --list
```

Current tooling includes logging/inspection, commissioning commands, parameter identification, plant fitting, and historical robust-control tools. Legacy Vq and H∞ utilities remain available as engineering/history artifacts, but they are **not** the active target attitude-control architecture.

See [`tools/README.md`](tools/README.md) for host workflows.

## Build

The current live firmware remains a native ESP-IDF project:

```text
ESP-IDF      v6.1
target       ESP-WROOM-32 / classic ESP32
runtime      native ESP-IDF C/C++ + CMake
```

Build:

```bash
idf.py build
```

The separately pinned Route-B build exists to integrate Arduino-ESP32 and SimpleFOC with the existing ESP-IDF component graph. Compatibility/link success is a software gate, not hardware commissioning evidence.

## Development gates

The current direction is deliberately staged:

1. keep generic fuzzy inference deterministic and tested;
2. keep `target_velocity`, wheel acceleration, and historical `Vq` as distinct units/interfaces in SITL;
3. prove near-upright fuzzy balance against the provisional local plant/servo uncertainty set;
4. independently commission the SimpleFOC motor velocity servo;
5. replace provisional motor-servo fixtures with identified behavior and uncertainty;
6. migrate target runtime AS5600/velocity/FOC/PWM ownership to SimpleFOC only after execution-domain gates are satisfied;
7. connect fuzzy attitude output through the `MotorControl` boundary;
8. extend capture/swing-up only when the corresponding model earns sufficient authority.

Hardware balancing remains blocked until the relevant software, simulation, and independent motor-commissioning gates are satisfied.

## Documentation

- [Full-fuzzy / SimpleFOC control boundary](docs/full-fuzzy-control.md)
- [Architecture](docs/architecture.md)
- [Hardware](docs/hardware.md)
- [Development](docs/development.md)
- [Motor bring-up](docs/motor-bringup.md)
- [SITL](tools/sitl/README.md)
- [Host tools](tools/README.md)
- [Parameter identification](tools/parameter_id/README.md)

> **README explains the system. Issues explain the journey. Code and evidence prove the current state.**

## License

MIT
