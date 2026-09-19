# Motor runtime

## Objective

Provide the reaction-wheel actuator path needed by the controller: native ESP-IDF MCPWM, AS5600 mechanical angle, automatic motor electrical calibration, and sensor-based voltage-mode FOC with commanded `Vq` as the higher-level input.

No phase-current loop is used because this board has no phase-current sensing.

## Hardware path

```text
ESP32 GPIO25/33/32
        ↓ MCPWM, 3-PWM
      EG2133
        ↓
  MOSFET bridge
        ↓
    2204 BLDC
        ↑
      AS5600
```

The PCB ties each `Moto_INx` net to both EG2133 `HINx` and active-low `LINx#`. The firmware therefore drives one PWM signal per phase.

`stop` commands the low-side zero vector. It removes commanded line-to-line voltage but is not a high-impedance disconnect.

## Runtime commands

```text
motor calibrate [amplitude_v] [electrical_hz] [turns]
motor config <pole_pairs> <sensor_dir> <offset_rad>
motor vq <volts>
motor status
motor stop

field <electrical_hz> <amplitude_v>
stop
status
telemetry [on|off]
help
```

`field` is the bounded open-loop rotating electrical field. It does not require a valid encoder magnet-status flag because it is an open-loop command.

## Automatic motor calibration

`motor calibrate` performs the commissioning sequence in firmware instead of requiring manual pole-pair calculations.

The routine:

1. applies a bounded d-axis alignment vector;
2. sweeps a known electrical angle through a configurable number of electrical turns;
3. reads the AS5600 multi-turn mechanical displacement;
4. derives the effective pole-pair count;
5. derives encoder/electrical direction;
6. derives the electrical-angle offset;
7. installs the resulting motor electrical configuration for the current runtime.

Default calibration parameters are intentionally low-energy:

```text
amplitude_v   0.6 V
electrical_hz 0.5 Hz
turns         4
```

They can be overridden by the command arguments without changing firmware.

## Voltage-mode FOC

After calibration or an explicit `motor config`, the runtime computes

```text
theta_e = sensor_dir * pole_pairs * theta_m + electrical_offset
```

and applies a d/q voltage transform with `Vd = 0` and commanded `Vq`.

The phase-voltage generator performs inverse Park/Clarke followed by common-mode injection for SVPWM-compatible three-phase modulation. The voltage vector is bounded by the configured motor voltage limit before being converted to MCPWM duties.

Higher-level control should use only:

```text
motor vq <volts>
```

The current bring-up voltage limit remains conservative at the board-defined `kBringupPhaseAmplitudeMaxV`.

## Encoder status

AS5600 status fields remain visible in `status` and telemetry, but magnet-status flags are diagnostic information rather than an artificial gate on open-loop actuation. Closed-loop FOC requires a valid angle sample.

Telemetry is off by default so the UART remains usable as an interactive console.
