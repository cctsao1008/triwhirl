# Development

This document covers the development environment and build workflow for TriWhirl.

## Toolchain

The ESP32 build is defined by `platformio.ini` and pins the toolchain and firmware dependencies used by CI.

Current pinned versions:

- pioarduino `platform-espressif32` release `55.03.311`
- Arduino-ESP32 `3.3.11`
- ESP-IDF libraries `5.5.5`
- SimpleFOC `2.4.0`
- target: ESP32 / ESP-WROOM-32

The pioarduino platform is used because the selected SimpleFOC ESP32 backend depends on the Arduino-ESP32 3.x / ESP-IDF 5.x generation.

## Build

From the repository root:

```bash
pio run
```

The default PlatformIO environment is `esp32_motor_bringup`.

## Serial monitor

The development console uses 115200 baud.

```bash
pio device monitor -b 115200
```

Motor-specific console commands and validation steps are documented in [motor-bringup.md](motor-bringup.md).

## Tests and CI

CI performs two independent checks:

1. native compilation and execution of host-testable C++ logic;
2. ESP32 PlatformIO cross-build using the pinned embedded toolchain.

Hardware-independent deterministic logic should live in `lib/triwhirl_core` where practical so the same implementation can be exercised by native tests and embedded builds.

See [architecture.md](architecture.md) for repository placement and dependency rules.
