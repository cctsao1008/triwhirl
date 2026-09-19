# Architecture

TriWhirl is organized as a native ESP-IDF application with project components and PC-side engineering tools.

## Top-level layout

```text
triwhirl/
├── main/                       application wiring / entry point
├── components/
│   ├── triwhirl_core/          platform-independent control/math code
│   └── triwhirl_hw/            ESP32 peripheral and board integration
├── tools/                      PC-side engineering/research tools
├── docs/                       documentation
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── LICENSE
```

## Component boundaries

`main/` owns application wiring and runtime orchestration.

`components/triwhirl_core/` contains project-owned deterministic control and signal-processing code that does not depend on ESP32 peripheral APIs.

`components/triwhirl_hw/` contains ESP32-specific board and peripheral integration using native ESP-IDF drivers.

`tools/` contains PC-side engineering utilities such as logging, calibration, identification, modeling, controller synthesis, and simulation when those tools are needed.

## Runtime direction

```text
sensors -> state estimation -> supervisor/control -> Vq -> motor modulation -> MCPWM -> bridge
```

Communication and UI paths stay outside the real-time control path. BLE/Web Bluetooth is implemented as an ESP-IDF NimBLE GATT transport when added.

## Dependency rule

Application wiring may depend on project components. Hardware-specific code may depend on ESP-IDF drivers. `triwhirl_core` must not depend on Arduino, PlatformIO, or ESP32 peripheral APIs.

The ESP32 runtime does not depend on Python or an online convex solver. Offline identification/modeling/synthesis tools may generate parameters consumed by the firmware.
