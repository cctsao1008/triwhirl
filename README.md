# TriWhirl

Robust control research platform for an ESP32-based reaction-wheel Reuleaux triangle.

TriWhirl is a clean-room redevelopment for the existing hardware. Seller firmware is not used as an implementation baseline.

## Current focus: motor-first bring-up

The first executable milestone is the reaction-wheel actuator path:

```text
ESP32 3-PWM -> EG2133 -> MOSFET bridge -> 2204 BLDC
                                      ^
                                      |
                                AS5600 angle
```

The initial firmware deliberately avoids assuming the motor pole-pair count. Instead it can generate a low-voltage rotating **electrical** field while logging AS5600 mechanical angle/velocity. The measured electrical/mechanical frequency ratio can then be used to determine pole pairs before sensor-based FOC is enabled.

This bring-up mode is not closed-loop FOC and is not the final controller.

## Toolchain

- PlatformIO `espressif32@7.1.3`
- Arduino framework supplied by that PlatformIO platform
- SimpleFOC `2.4.0`
- ESP32 / ESP-WROOM-32 target

Dependencies are pinned intentionally.

## Build

```bash
pio run
```

## Motor bring-up console

After flashing, open a 115200 baud serial monitor.

```text
help
status
field <electrical_hz> <amplitude_v>
stop
```

Example low-energy command:

```text
field 2 0.5
```

`field` is disabled at boot. The initial software ceiling is 1.5 V field amplitude and exists only for controlled bring-up; it is not yet a measured hardware operating limit.

## Scope constraints

- Existing PCB only; no hardware redesign.
- No phase-current sensing.
- Final balance control: robust H-infinity state feedback synthesized offline via LMI.
- No LQR implementation or benchmark.
- USB/UART is the primary development interface.
- Classic Bluetooth SPP may later be used for optional telemetry/debug.
- Wi-Fi is out of scope.

## License

MIT.
