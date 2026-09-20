# Runtime refactor A2

Issue: #32

A2 removes runtime source-inclusion composition while preserving the validated
control behavior and command wire protocol.

## Current active composition

```text
runtime_main.cpp         explicit app entry + Core-1 realtime orchestration
runtime_state.cpp/.hpp   shared runtime state and bring-up/control helpers
runtime_diagnostics.*    bounded diagnostic snapshot enrichment + Core-0 AS5600 health
```

The historical composition chain is gone:

```text
runtime_control.cpp -> runtime_main.cpp -> app_main.cpp
```

`runtime_control.cpp` and legacy `app_main.cpp` are no longer present. There are
no `.cpp` source inclusions and no symbol-renaming shims.

## Explicit runtime boundaries

```text
Core 0                                           Core 1
------                                           ------
runtime_encoder_acquisition                ->    runtime_main
  AS5600 raw read                                wheel-state commit
  bounded result queue                           miss/safety policy

runtime_supervisor_io
  UART0 wired development/service CLI
  NimBLE GATT RX-characteristic ingress
  line assembly
       |
       v
runtime_command_parser                     ->    runtime_main
  mutation grammar                               at most one queued event / RT iteration
  numeric parsing                                typed command execution
  usage errors                                   no raw command strings
  fixed-size RuntimeCommand

runtime_supervisor_io                      <-    runtime_snapshot
  read-only formatting                           latest complete snapshot
  status / motor status                          state copied on Core 1
  imu / log / attitude / fault / timing          non-blocking overwrite publication
  telemetry query

runtime_supervisor_io -- Core 0 live read --> AS5600 status
BLE GATT status -- direct transport state --> runtime_supervisor_io
```

BLE is a NimBLE GATT transport, not UART. RX characteristic writes are buffered
inside the BLE component and drained by `runtime_supervisor_io`; TX uses GATT
notifications. UART0 remains a separate wired development/service CLI. Neither
transport has realtime authority.

`runtime_supervisor_io` owns ingress/framing while `runtime_command_parser` owns
mutation command text, aliases, numeric parsing, and usage validation. No raw
command string or line buffer crosses into Core 1.

The supervisor mailbox is depth-limited and non-blocking. Mailbox saturation is
reported as `ERR command mailbox full`; unknown commands are rejected on Core 0.
The snapshot channel is depth one with overwrite/peek semantics, and the first
snapshot is published before supervisor ingress starts.

`RuntimeCommand` is a trivially-copyable fixed-size mailbox record and is bounded
to 64 bytes by compile-time assertions. Motor, IMU, swing, timing-profile,
logger, and attitude mutations arrive on Core 1 only as typed records. Original
aliases, usage errors, and swing ownership behavior are preserved.

## Read-only diagnostics moved out of realtime command execution

Core 0 now owns the externally reachable read-only forms:

```text
status
motor status
imu / imu status
log / log status
attitude / attitude status
fault / fault status
ble / ble status
timing / timing status
telemetry
help
```

`status` combines the latest bounded `RuntimeSnapshot` with an AS5600 status
transaction performed from the Core-0 supervisor domain. The AS5600 acquisition
worker and diagnostic read use the driver's existing mutex, so the live magnet
health transaction no longer runs in the Core-1 deadline.

MPU6050 identity is probed during driver initialization and cached inside the
driver. `readWhoAmI()` is therefore a cached read after successful init; `imu
status` does not issue a command-triggered I2C transaction.

Logger status is copied into the diagnostic snapshot at a 20 ms supervisory
cadence instead of querying logger state because a user typed `log status`.
Read-only formatting then happens on Core 0 using the cached snapshot.

The existing wire formats for aggregate status, IMU status, and log status are
preserved.

## Composition cleanup: complete

The following transitional mechanisms have been removed:

```text
SupervisorInputEventType::kCommand
raw command line crossing into Core 1
handleSupervisorCommand(event.line)
legacy swing/timing string parsers
updateEncoder/updateImu rename shims
initEncoderBus/initImuBus rename shims
runtime_control.cpp wrapper
app_main rename shim
all .cpp source inclusion
```

`runtime_state.cpp/.hpp` now explicitly owns the state and helpers that were
previously visible only because `app_main.cpp` was textually included. CMake
compiles every runtime service as a normal translation unit. The architecture
contract requires:

```text
cpp_includes = 0
renaming_shims = 0
app_main owner = runtime_main.cpp
legacy app_main.cpp absent
runtime_control.cpp absent
runtime_diagnostics.cpp explicit
```

## Remaining A2 debt

The command-ingress and read-only diagnostic boundaries are now substantially
cleaner. Remaining work is concentrated in outbound ownership:

- `swing status` and `timing profile status` are still formatted from
  realtime-owned structures;
- mutation command success/error responses are still formatted by Core 1;
- periodic telemetry, swing transition events, calibration/fault events, and
  timing-profile summaries still originate as text from Core 1.

Transport transmission itself is already buffered: UART text drains through the
Core-0 console TX task and BLE `write()` feeds the BLE TX stream consumed by the
Core-0 BLE TX task. The remaining problem is therefore **where protocol text is
constructed**, not raw transport notification ownership.

The next major slice should introduce bounded structured Core-1 -> Core-0 result
and event records, then move protocol formatting into the supervisor domain. It
must not add blocking transport calls or output mutexes to the realtime task.

## Target

```text
runtime_main.cpp          app entry + realtime orchestration
runtime_state.cpp/.hpp    state/control primitives
runtime_supervisor_io.*   Core-0 transport/framing/formatting
runtime_command_parser.*  Core-0 mutation grammar
runtime_snapshot.*        latest read-only runtime state
runtime_diagnostics.*     bounded diagnostic state + Core-0 sensor diagnostics
runtime_encoder_acquisition.*
runtime_platform.*
runtime_release.*
```

## Acceptance

- no `.cpp` source inclusion;
- no symbol-renaming or generic ESP-IDF API interception;
- one explicit application entry path;
- one explicit realtime control task;
- UART development ingress and BLE GATT ingress remain outside realtime control;
- command ingress is fixed-size, bounded, non-blocking, and explicitly rejects
  overflow;
- no raw command strings cross into realtime;
- no command-triggered diagnostic I2C runs in the realtime deadline;
- externally reachable read-only status formatting is supervisor-owned;
- protocol formatting for realtime command results/events moves through bounded
  non-blocking Core-1 -> Core-0 records before A2 closes;
- command protocol and identification behavior remain preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
