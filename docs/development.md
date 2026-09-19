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
components/triwhirl_core/   platform-independent deterministic C++
components/triwhirl_hw/     ESP32 peripheral and board integration
test/                       native host tests
tools/                      PC-side engineering/research tools
docs/                       documentation
```

The build follows ESP-IDF's native component model. Hardware-independent control mathematics stays in `triwhirl_core` so it can be compiled both by ESP-IDF and by host-side tests.

## CI

CI runs native C++ tests and a separate firmware build using Espressif's official ESP-IDF CI action pinned to v6.1.
