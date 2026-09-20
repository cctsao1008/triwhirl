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
  complete command grammar                  at most one queued event / RT iteration
  numeric parsing                           typed command execution
  usage errors                              no raw command strings
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
owns command text, numeric parsing, aliases, and usage validation. No raw command
string or line buffer crosses into Core 1 anymore.

The supervisor mailbox is depth-limited and non-blocking. When full, the newest
command/event is rejected and the transport reports `ERR command mailbox full`
rather than blocking either domain. Unknown commands are rejected in the
supervisor domain with `ERR unknown command`.

The snapshot channel is depth one and uses overwrite/peek semantics. Core 1
publishes the latest complete snapshot without blocking; Core 0 reads a complete
copy without consuming it. `attitude status`, `fault status`, `timing status`,
and the read-only `telemetry` query are snapshot-backed on Core 0. `ble status`
is handled entirely on Core 0 because its authoritative state belongs to the BLE
GATT transport itself. Static `help` formatting is supervisor-owned as well. The
first snapshot is published before supervisor ingress starts.

`runtime_command.hpp` now represents the complete remaining command surface that
needs realtime-owned state. This includes motor, IMU, swing, timing-profile, and
logger lifecycle operations plus status requests that still depend on legacy
runtime-owned state. Numeric conversion for all payload-bearing commands occurs
only in `runtime_command_parser` on Core 0.

The original command-family aliases are preserved (`imu`, `log`, `swing`, and
`timing profile` still resolve to their status forms; `attitude`, `timing`,
`fault`, and `ble` status aliases remain supervisor-owned). Existing usage-error
strings and the swing ownership policy are preserved.

## Legacy string path: removed

The following transitional path has been deleted:

```text
SupervisorInputEventType::kCommand
char line[128] crossing into Core 1
handleSupervisorCommand(event.line)
```

`SupervisorInputEvent` now carries only typed `RuntimeCommand` records or small
supervisor bookkeeping events. CI explicitly fails if the raw string path is
reintroduced.

## Remaining A2 debt

Command parsing is no longer part of realtime execution, but A2 is not complete
yet. Remaining work is structural and output/diagnostic ownership:

- `runtime_control.cpp -> runtime_main.cpp -> app_main.cpp` source inclusion;
- five legacy symbol-renaming shims in `runtime_main.cpp`;
- status/IMU formatting still executed from Core 1 for commands whose state has
  not yet been fully represented in `RuntimeSnapshot`;
- `status` still refreshes AS5600 health and `imu status` still performs a
  WHO_AM_I diagnostic I2C transaction in the realtime translation unit;
- command responses and some telemetry/event formatting still originate on Core
  1, so final single-writer supervisor egress is still pending.

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
- no raw command strings cross into realtime;
- string parsing/formatting is removed from realtime control before A2 closes;
- read-only diagnostics consume bounded snapshot/transport state rather than
  live cross-core state;
- command protocol and identification behavior preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
