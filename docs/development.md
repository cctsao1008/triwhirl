# Development

TriWhirl uses the official Espressif ESP-IDF toolchain directly.

## Firmware toolchain

- ESP-IDF `v6.1`
- target: classic ESP32 / ESP-WROOM-32
- native ESP-IDF C/C++ components and CMake build

There is no PlatformIO, Arduino core, or SimpleFOC runtime dependency.

## Build

Activate an ESP-IDF v6.1 environment, then from the repository root run:

```bash
idf.py build
```

## Flash and monitor

For the current CH340 connection on Windows, replace the port as needed:

```bash
idf.py -p COM28 flash monitor
```

Exit the monitor with the ESP-IDF monitor escape sequence shown by `idf.py monitor`.

## Repository structure

```text
main/                       application wiring / entry point
components/triwhirl_core/   platform-independent control/math code
components/triwhirl_hw/     ESP32 peripheral and board integration
tools/                      PC-side engineering/research tools
docs/                       documentation
```

The build follows ESP-IDF's native component model. `main/` wires the application together; reusable project code is kept in components.

## CI

CI performs a native ESP-IDF cross-build with Espressif's official CI action pinned to ESP-IDF v6.1. Hardware behavior is validated on the actual TriWhirl board during bring-up rather than through a permanent duplicate host-test suite.
