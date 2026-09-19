# Architecture

TriWhirl is organized as a native ESP-IDF application with project components and PC-side engineering tools.

## Top-level layout

```text
triwhirl/
├── main/                       application wiring / entry point
├── components/
│   ├── triwhirl_core/          platform-independent estimation/control/safety math
│   ├── triwhirl_hw/            ESP32 peripheral and board integration
│   └── triwhirl_ble/           native ESP-IDF NimBLE transport
├── tools/                      logging, WebUI, identification, synthesis tools
├── docs/                       stable technical documentation
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── LICENSE
```

## Component boundaries

`main/` owns application wiring and runtime orchestration.

`components/triwhirl_core/` contains project-owned deterministic estimation, safety, control, and signal-processing code that does not depend on ESP32 peripheral APIs.

`components/triwhirl_hw/` contains ESP32-specific board and peripheral integration using native ESP-IDF drivers.

`components/triwhirl_ble/` contains the native ESP-IDF NimBLE GATT transport. It is an engineering/debug transport and does not own control state.

`tools/` contains PC/browser-side engineering utilities for logging, Web Bluetooth UI, parameter identification, modeling, controller synthesis, and simulation as needed.

## Runtime direction

```text
AS5600 + MPU6050
       ↓
state estimation
       ↓
safety / supervisor authority
       ↓
controller -> Vq
       ↓
voltage-mode motor modulation
       ↓
native MCPWM -> EG2133 -> BLDC
```

The deterministic sensor/estimator/motor path runs at 1 kHz. Communication and UI paths stay outside that real-time authority:

```text
                 ┌─ CH340 / UART
runtime protocol ┤
                 └─ NimBLE GATT -> Web Bluetooth -> WebUI
```

Both transports use the same line-oriented command and telemetry application protocol. Transport disconnects do not own or reset motor/control state.

## Safety authority

The safety layer is project-owned and independent of UART, BLE, or browser state. A detected active fault latches its cause and drives the motor to the board's defined low-side zero vector. Because of the EG2133 wiring, this state is not described as high impedance.

Physical limits that still require measurement or later controller design remain explicit unknowns rather than guessed constants.

## Dependency rule

Application wiring may depend on project components. Hardware-specific code may depend on ESP-IDF drivers. `triwhirl_core` must not depend on Arduino, PlatformIO, or ESP32 peripheral APIs.

The ESP32 runtime does not depend on Python or an online convex solver. Offline identification/modeling/H∞ synthesis tools generate models or coefficients consumed by the firmware.
