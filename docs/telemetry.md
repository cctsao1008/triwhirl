# Telemetry protocol

TriWhirl uses the existing CH340 UART as the primary engineering transport. The runtime command channel is line-oriented ASCII at 115200 baud, 8-N-1. The same application protocol is mirrored over the BLE GATT transport used by the Web Bluetooth UI. Telemetry is emitted as one CSV record per line and is queued outside the real-time control path.

## Schema v2

The firmware declares the telemetry field order at boot:

```text
telemetry_fields,t_us,mode,vq_v,e_angle_rad,e_hz,status_ok,sample_ok,mag,raw,unwrapped_count,angle_rad,unwrapped_rad,vel_rad_s,vel_inst_rad_s,vel_valid,read_errors,imu_ok,ax,ay,az,gx,gy,gz,imu_read_errors,attitude_ok,theta_rad,theta_rate_rad_s,accel_weight,loop_exec_us,loop_max_exec_us,loop_overruns,fault_mask
```

This exact header identifies telemetry schema **v2**. Data records use the same field order and the `telemetry` prefix:

```text
telemetry,<t_us>,<mode>,<vq_v>,...
```

`fault_mask` is the latched runtime safety-fault bit mask. A nonzero value means motor actuation is inhibited until the cause is no longer present and the latch is explicitly cleared.

The host capture tool writes `schema_version=2` as the first CSV column so recorded logs remain self-describing even though the embedded line format stays compact.

## Commands

The command channel is newline terminated. Current commands include motor commissioning/actuation, IMU and attitude status/configuration, timing status, safety-fault status, BLE status, and telemetry control. Human-readable commands remain intentionally usable from a normal serial terminal.

Safety state can be inspected with:

```text
fault status
```

A latched fault can be cleared only while the motor is stopped and the directly checkable cause is no longer present:

```text
fault clear
```

Telemetry streaming is controlled with:

```text
telemetry on
telemetry off
```

## Capture

From the ESP-IDF PowerShell environment:

```powershell
python tools/logging/capture.py COM28
```

The tool enables telemetry, validates the known schema when the boot header is seen, writes an analysis-ready CSV file, and disables telemetry on normal exit or Ctrl+C.

Useful options:

```powershell
python tools/logging/capture.py COM28 -o logs/run.csv
python tools/logging/capture.py COM28 --duration 30
```

The capture path is diagnostic only. Serial output backpressure is handled by the firmware's low-priority TX path and cannot block the 1 kHz control task.
