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
  <em>Measure first. Capture carefully. Synthesize deliberately.</em>
</p>

<p align="center">
  🔺 Geometry &nbsp;·&nbsp; 🌀 Momentum &nbsp;·&nbsp; 🧠 Control &nbsp;·&nbsp; 🔬 Validate
</p>

TriWhirl is a native ESP-IDF control and system-identification platform for a reaction-wheel-stabilized Reuleaux triangle. It keeps sensing evidence, estimated state, stand-up logic, controller intent, actuator limits, runtime authority, transport, and post-run analysis deliberately separate so that the control stack cannot claim more certainty—or more authority—than the measured hardware supports.

> **Curved triangle. Hard evidence. No host-assisted balance.**

---

## 🧠 Architecture

The realtime path is firmware-owned from sensing through actuation:

```text
AS5600 + MPU6050
       ↓
SensorFrame
       ↓
state estimation
       ↓
safety / supervisor authority
       ↓
stand-up / balance controller
       ↓
bounded Vq command
       ↓
native MCPWM
       ↓
EG2133 -> BLDC -> reaction wheel
```

Engineering transports remain outside realtime control authority:

```text
                 ┌─ CH340 / UART
runtime protocol ┤
                 └─ NimBLE GATT -> host toolbox
```

The Reuleaux-triangle upright coordinate is 120-degree periodic. The three physical vertices therefore share one local balance coordinate and one local control formulation rather than being treated as three unrelated equilibria.

## 🧰 Firmware shape

```text
triwhirl/
├── main/                       application wiring / runtime orchestration
├── components/
│   ├── triwhirl_core/          estimation, geometry, control, safety math
│   ├── triwhirl_hw/            ESP32 board / peripheral integration
│   └── triwhirl_ble/           native ESP-IDF NimBLE transport
├── tools/                      commissioning, logging, ID, fitting, synthesis
├── docs/                       durable technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
└── README.md
```

`triwhirl_core` is platform-independent project logic and does not depend on Arduino, PlatformIO, SimpleFOC, or ESP32 peripheral APIs. Hardware ownership stays in `triwhirl_hw`; application composition stays in `main/`.

## 🎯 Control lanes

TriWhirl deliberately keeps the commissioning controller and the research controller conceptually separate.

```text
                    autonomous stand-up
                           │
                   reaction-wheel swing-up
                           │
                     upright capture
                           │
              ┌────────────┴────────────┐
              │                         │
     commissioning lane          research lane
              │                         │
  state feedback / velocity PI     identified plant
              │                         │
   bounded Vq + anti-windup         H∞ synthesis
              │                         │
     hardware reference         guarded deployment
```

The current autonomous stand-up path uses a vendor-aligned state-feedback / velocity-PI controller as a **commissioning baseline**. It exists to validate sensor coordinates, motor sign, capture behavior, actuator authority, timing, and traceability on the real hardware. It is not a claim that LQR has replaced the research objective.

The research path remains:

```text
measurement
    ↓
local plant identification
    ↓
model + uncertainty
    ↓
H∞ synthesis
    ↓
guarded near-upright deployment
    ↓
hardware comparison against the commissioning baseline
```

## 🌀 Stand-up architecture

The autonomous hardware sequence is:

```text
SwingHigh / SwingLow
        ↓
   capture window
        ↓
      Balance
        ↓
state-feedback wheel-velocity target
        ↓
velocity PI
        ↓
       Vq
```

The commissioning controller includes capture/release hysteresis, bounded wheel-velocity targets, bounded `Vq`, actuator slew limiting, velocity-PI reset on recapture, and conditional anti-windup. These mechanisms prevent a failed capture from silently carrying saturated integral state into the next attempt.

Controller tuning remains trace-driven. Temporary gain values and experiment-specific tuning history belong in code, captures, and Issues rather than being promoted into permanent physical truth in this README.

## 🛡️ Actuation authority

```text
controller intent
      ↓
limit / safety policy
      ↓
authorized Vq
      ↓
3-PWM electrical realization
      ↓
EG2133 gate driver
      ↓
reaction-wheel motor
```

UART, BLE, Python, plotting, and browser/host state do not grant motor authority. Transport disconnects do not own the realtime control state.

Because the board ties each EG2133 active-high high-side command and active-low low-side command to one complementary MCU signal, a zero-duty command produces the board-defined low-side zero vector rather than a guaranteed high-impedance motor disconnect. Electrical semantics are therefore documented separately from abstract controller intent.

## 🔬 Validation and evidence

TriWhirl treats captured hardware data as evidence, not decoration.

The dedicated stand-up trace records controller state and measured sample timing in firmware, then transfers the completed capture to the host for analysis. Plotting and disk I/O occur after the realtime experiment rather than inside the control path.

A normal commissioning run is:

```powershell
python tools/twtool.py control standup --duration 10 --plot
```

The host produces the authoritative raw trace plus decoded analysis artifacts:

```text
.twtrace    binary stand-up evidence
.csv        decoded samples
.json       provenance / acceptance metadata
.png        post-run visualization
```

Trace acceptance checks framing, CRC, sequence continuity, transport/ring drops, timing integrity, firmware provenance, and host/firmware revision agreement. The trace stores measured `dt_us`; analysis does not invent a perfect sample period after the fact.

Existing runs can be replotted independently:

```powershell
python tools/twtool.py log plot-standup artifacts/standup/<capture>.twtrace
```

## 📏 Physical parameter gate

Unknown physical values remain explicit unknowns until measured or identified. Current examples include final safe continuous/transient `Vq`, hard reaction-wheel speed limits, bus behavior under load, battery ADC transfer function, and final robust-control uncertainty bounds.

```text
physical measurement / experiment
              ↓
      identified evidence
              ↓
       admissible parameter
              ↓
      controller / safety use
```

A convenient number from a seller sketch, simulation, or temporary tuning run does not become a TriWhirl physical fact merely because the controller happens to move.

## 🧪 Identification and H∞ workflow

Host-side engineering tools support actuator, body, and swing identification without moving realtime control authority off the ESP32.

```text
hardware experiment
      ↓
firmware-owned synchronized capture
      ↓
host decode / inspect
      ↓
plant fitting
      ↓
uncertainty characterization
      ↓
H∞ synthesis
      ↓
firmware coefficients / guarded trial
```

The ESP32 runtime does not depend on Python or an online convex solver. Identification, fitting, and controller synthesis are offline engineering operations whose outputs are consumed by deterministic firmware.

## 🧰 Host toolbox

`twtool` is the canonical host entry point:

```powershell
python tools/twtool.py --help
python tools/twtool.py --list
```

Current command groups:

```text
log      firmware logging, download, decode, inspection, plotting
control  autonomous stand-up and guarded balance deployment
id       actuator / body / swing identification experiments
fit      model fitting from measured data
```

Examples:

```powershell
python tools/twtool.py control standup --duration 10 --plot
python tools/twtool.py id swing --probes 12 -o artifacts/auto-swing-id-01.csv
python tools/twtool.py log inspect artifacts/run-01.twlog
```

See [`tools/README.md`](tools/README.md) for the complete host workflow.

## 🔧 Build

TriWhirl uses the official Espressif toolchain directly:

```text
ESP-IDF      v6.1
target       ESP-WROOM-32 / classic ESP32
runtime      native ESP-IDF C/C++ + CMake
```

Build:

```bash
idf.py build
```

Flash the currently verified Windows setup:

```bash
idf.py -p COM28 flash
```

The board uses a CH340 USB/UART path and requires manual ESP32 ROM download-mode entry on the verified hardware. See [`docs/development.md`](docs/development.md) for the exact procedure.

## 📚 Documentation

- [Architecture](docs/architecture.md)
- [Hardware](docs/hardware.md)
- [Development](docs/development.md)
- [Motor bring-up](docs/motor-bringup.md)
- [Host tools](tools/README.md)
- [Parameter identification](tools/parameter_id/README.md)

## 📚 Documentation principle

> **README explains the system. Issues explain the journey. Code proves the current state.**

README and durable documentation explain architecture, control boundaries, runtime authority, hardware semantics, validation interpretation, and research workflow. GitHub Issues preserve experiments, tuning, temporary constraints, implementation steps, and closure records. Code, configuration, captured evidence, and tests remain the authoritative proof of executable behavior.

## License

MIT
