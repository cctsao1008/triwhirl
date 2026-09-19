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

The current board exposes its CH340 download path through USB Type-C, but entering the ESP32 ROM download mode is manual on this hardware.

For the verified board procedure:

1. connect the board to the PC through USB Type-C and power the board;
2. press and hold the board's **Download** button;
3. press the **Reset** button to enter download mode;
4. flash through the CH340 COM port;
5. after flashing, release **Download** if it is still held and press **Reset** once to run the application.

For the current Windows connection:

```bash
idf.py -p COM28 flash
```

Then start the monitor separately:

```bash
idf.py -p COM28 monitor
```

The port number may differ on another PC. The board may require the manual Reset press after flashing even if esptool reports a hard reset through RTS.

Exit the ESP-IDF monitor with `Ctrl+]`.

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
