# TriWhirl

TriWhirl is an ESP32-based research platform for autonomous swing-up and robust balancing of a reaction-wheel Reuleaux triangle.

The project is built around a native ESP-IDF realtime runtime, project-owned estimation/control code, and a PC-side engineering toolbox for commissioning, logging, identification, fitting, plotting, and controller development.

> Current status: active hardware commissioning. The runtime, sensing, motor drive, trace capture, swing-up/balance handoff, and host analysis path are operational. Near-upright control is being tuned from measured hardware traces rather than treated as a finished controller.

## System overview

```text
AS5600 + MPU6050
       |
       v
state estimation
       |
       v
safety / supervisor
       |
       v
swing-up / balance controller
       |
       v
reaction-wheel velocity target
       |
       v
velocity PI -> Vq
       |
       v
native MCPWM -> EG2133 -> BLDC
```

The deterministic sensor / estimator / control path runs at 1 kHz. UART, BLE, logging, plotting, and other engineering interfaces stay outside realtime control authority.

The upright geometry is treated as 120-degree periodic, so the same local balance law can be applied around all three physical vertices.

## Current control work

The autonomous stand-up path currently consists of:

1. reaction-wheel swing-up using the proven TRC-V1.1 handoff structure;
2. a guarded capture window around the local upright;
3. an outer state-feedback law producing a reaction-wheel velocity target;
4. an inner velocity PI producing `Vq`;
5. bounded actuator output, slew limiting, capture hysteresis, PI reset on recapture, and conditional anti-windup.

The current commissioning baseline intentionally uses conservative trace-tuned gains and limits:

```text
velocity target limit : +/-60 rad/s
Vq limit              : +/-3.0 V
velocity output ramp  : 1000 V/s
balance capture       : 9 deg
balance release       : 12 deg
```

The repository also contains a guarded H-infinity near-upright deployment path. Controller development remains measurement-driven: firmware records authoritative realtime traces and the host performs post-run analysis rather than participating in the control loop.

## Firmware

TriWhirl uses the official Espressif toolchain directly:

- ESP-IDF `v6.1`
- target: ESP-WROOM-32 / classic ESP32
- native ESP-IDF C/C++ components and CMake
- no PlatformIO runtime
- no Arduino core runtime
- no SimpleFOC runtime dependency

Build from an activated ESP-IDF v6.1 environment:

```bash
idf.py build
```

For the currently verified Windows setup:

```bash
idf.py -p COM28 flash
idf.py -p COM28 monitor
```

The board uses a CH340 USB/UART path and requires manual ESP32 download-mode entry on the verified hardware. See [Development](docs/development.md) for the exact procedure.

## Host toolbox

`twtool` is the canonical PC-side entry point:

```powershell
python tools/twtool.py --help
python tools/twtool.py --list
```

The main command groups are:

```text
log      firmware logging, download, decode, inspection, plotting
control  autonomous stand-up and guarded balance deployment
id       actuator/body/swing identification experiments
fit      model fitting from measured data
```

### Stand-up commissioning

Run an autonomous hardware trial and save the dedicated stand-up trace:

```powershell
python tools/twtool.py control standup --duration 10 --plot
```

The command captures the firmware-owned 1 kHz binary trace into host RAM during the run, then writes the raw `.twtrace`, decoded `.csv`, `.json` metadata, and optional PNG only after the motor has stopped.

The trace carries measured sample timing and firmware git provenance. Post-run acceptance checks validate framing, CRC, sample continuity, transport drops, timing integrity, and host/firmware revision agreement.

Existing captures can be replotted independently:

```powershell
python tools/twtool.py log plot-standup artifacts/standup/<capture>.twtrace
```

See [Host tools](tools/README.md) for the full workflow.

## Repository layout

```text
triwhirl/
├── main/                       application wiring and runtime orchestration
├── components/
│   ├── triwhirl_core/          platform-independent estimation/control/safety
│   ├── triwhirl_hw/            ESP32 board and peripheral integration
│   └── triwhirl_ble/           native ESP-IDF NimBLE transport
├── tools/                      host-side commissioning and research toolbox
├── docs/                       technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
└── README.md
```

## Design boundaries

TriWhirl deliberately separates realtime control from engineering transport and analysis:

- the ESP32 owns sensing, estimation, safety, control, and synchronized acquisition;
- BLE and UART are configuration/debug transports, not realtime control paths;
- Python tooling performs experiment orchestration and post-run analysis only;
- hardware-specific ESP-IDF code stays outside `triwhirl_core`;
- unknown physical limits remain explicit until they are measured or identified.

This separation is intentional: reproducible traces and hardware evidence are preferred over hidden assumptions or host-assisted timing.

## Documentation

- [Architecture](docs/architecture.md)
- [Hardware](docs/hardware.md)
- [Development](docs/development.md)
- [Motor bring-up](docs/motor-bringup.md)
- [Host tools](tools/README.md)
- [Parameter identification](tools/parameter_id/README.md)

## License

MIT
