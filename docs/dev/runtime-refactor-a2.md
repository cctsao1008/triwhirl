# Runtime refactor A2

Issue: #32

A2 removes the remaining source-inclusion composition without changing the
validated control behavior.

## Current active path

```text
runtime_control.cpp
  -> runtime_main.cpp
     -> app_main.cpp
```

## Explicit runtime boundaries already extracted

```text
Core 0                                      Core 1
------                                      ------
runtime_encoder_acquisition           ->    runtime_control
  AS5600 raw read                           wheel-state commit
  bounded result queue                      miss/safety policy

runtime_supervisor_io
  UART0 wired development/service CLI
  NimBLE GATT RX-characteristic ingress
  line assembly
       |
       v
runtime_command_parser                ->    runtime_control
  command grammar                           at most one queued event / RT iteration
  numeric parsing                           typed mutation execution
  usage errors                              legacy string fallback (transitional)
  fixed-size RuntimeCommand

runtime_supervisor_io                 <-    runtime_snapshot
  read-only formatting                      latest complete snapshot
  attitude/fault/timing/telemetry status    non-blocking overwrite publication
  BLE GATT status direct from BLE transport
  static help text
```

BLE is a NimBLE GATT transport, not UART. The RX characteristic accepts GATT
writes and the BLE component places those payload bytes into an internal stream
buffer; `runtime_supervisor_io` drains that buffer. TX uses the GATT notify
characteristic. UART0 remains intentionally separate as a wired development and
service CLI during bring-up. Neither transport has realtime authority.

Transport ownership and command grammar are separate services:
`runtime_supervisor_io` owns ingress/framing while `runtime_command_parser`
owns migrated command text, numeric parsing, and usage validation. This keeps
transport code from becoming the permanent command parser and gives the legacy
Core-1 string path a single replacement boundary.

The supervisor mailbox is depth-limited and non-blocking. When full, the newest
command/event is rejected and the transport reports `ERR command mailbox full`
rather than blocking either domain.

The snapshot channel is depth one and uses overwrite/peek semantics. Core 1
publishes the latest complete snapshot without blocking; Core 0 reads a complete
copy without consuming it. `attitude status`, `fault status`, `timing status`,
and the read-only `telemetry` query are snapshot-backed on Core 0. `ble status`
is also handled entirely on Core 0 because its authoritative state belongs to
the BLE GATT transport itself. Static `help` formatting is supervisor-owned as
well. The initial snapshot is published before supervisor ingress starts, so
these commands do not race the first snapshot publication at startup.

`runtime_command.hpp` defines the fixed-size supervisor/realtime mutation
contract. Commands now parsed on Core 0 and executed on Core 1 without string
interpretation include:

- no-payload commands: `motor stop`, `stop`, `swing abort`, `swing start`,
  `timing reset`, `timing profile on`, `timing profile off`,
  `timing profile reset`, `fault clear`, `telemetry on`, `telemetry off`;
- payload commands: `motor vq <volts>`,
  `motor config <pole_pairs> <sensor_dir> <offset_rad>`,
  `motor calibrate [amplitude_v] [electrical_hz] [turns]`,
  `field <electrical_hz> <amplitude_v>`, `attitude reset [angle_rad]`,
  `imu calibrate [samples]`,
  `imu map <sin_axis> <cos_axis> <gyro_axis> <sin_sign> <cos_sign> <gyro_sign>`,
  and `swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>`.

The payload forms use fixed-size POD fields in `RuntimeCommand`; `atoi`, `strtof`,
`strtod` and `strtoul` parsing remains entirely in `runtime_command_parser` on
Core 0. Realtime still performs state-dependent validation, safety admission and
mutation so actuator/sensor ownership does not move across cores. Swing-config
duration conversion is also completed on Core 0; realtime still enforces the
motor-voltage limit and delegates structural validation to `SwingIdRunner`.

The parser has a host-side contract test in
`tools/tests/runtime_command_parser_test.cpp`. CI compiles the parser directly
with the host compiler and verifies command type selection, payload conversion,
default arguments, swing-duration conversion, usage errors, and the remaining
legacy-not-matched boundary before the ESP-IDF build begins.

The original swing-ownership policy is preserved. While identification owns
realtime actuation, the swing command family itself still reaches its existing
state-dependent checks, timing-profile control remains available as before, and
`telemetry off` remains allowed. Other migrated mutations are rejected by the
same swing-ownership policy.

This remains an incremental B2 slice. Commands that are not yet migrated still
enter Core 1 through the legacy string event, so string parsing has not yet been
fully removed from realtime. The legacy event is migration debt, not a target
compatibility layer; it should disappear once the remaining command grammar has
been represented as typed commands or supervisor read-only operations.

## Target

```text
runtime_startup.cpp
runtime_control.cpp
runtime_supervisor.cpp
runtime_state.cpp/.hpp
runtime_platform.cpp/.hpp
runtime_release.cpp/.hpp
```

Supporting bounded services may remain separate when they have one clear
hardware/concurrency responsibility, such as:

```text
runtime_command.hpp
runtime_command_parser.cpp/.hpp
runtime_encoder_acquisition.cpp/.hpp
runtime_supervisor_io.cpp/.hpp
runtime_snapshot.cpp/.hpp
```

## Acceptance

- no `.cpp` source inclusion;
- no symbol-renaming or generic ESP-IDF API interception;
- one explicit application entry path;
- one explicit realtime control task;
- UART development ingress and BLE GATT ingress stay outside realtime control;
- UART0 may remain as a wired development/service transport, but it does not
  bypass the same Core-0 parser and typed command boundary used by BLE GATT;
- command ingress is fixed-size, bounded, non-blocking, and explicitly rejects
  overflow;
- migrated mutating commands cross as typed `RuntimeCommand` records;
- string parsing/formatting is removed from realtime control before A2 closes;
- read-only diagnostics consume bounded snapshot/transport state rather than
  live cross-core state;
- command protocol and identification behavior preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
