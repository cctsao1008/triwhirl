# Runtime refactor A2

Issue: #32

A2 removes runtime source-inclusion composition while preserving the validated
control behavior and command wire protocol.

## Current active composition

```text
runtime_main.cpp             explicit app entry + Core-1 realtime orchestration
runtime_state.cpp/.hpp       shared runtime state and bring-up/control helpers
runtime_diagnostics.*        bounded diagnostic snapshot enrichment + Core-0 AS5600 health
runtime_command_parser.*     Core-0 command grammar and numeric parsing
runtime_reply.hpp            synchronous command/swing/timing-profile records
runtime_state_event.hpp      asynchronous state/calibration/fault records
runtime_telemetry.hpp        fixed-size periodic telemetry frame
runtime_profile_report.hpp   fixed-size parallel-profile report
runtime_egress.hpp           unified Core-1 -> Core-0 publication API
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
  command grammar                                at most one queued command / RT iteration
  numeric parsing                                typed command execution
  usage errors                                   no raw command strings
  fixed-size RuntimeCommand

runtime_supervisor_io                      <-    runtime_snapshot
  read-only formatting                           latest complete snapshot
  status / motor / IMU / log                     non-blocking overwrite publication
  attitude / fault / timing / telemetry query

runtime_supervisor_io                      <-    unified RuntimeEgressRecord FIFO
  protocol formatting                            RuntimeReply
  command result + prompt                        RuntimeStateEvent
  swing/timing-profile reports                   RuntimeTelemetryFrame
  fault/calibration events                       RuntimeProfileReport
  telemetry + parallel profile                   zero-timeout Core-1 publication

runtime_supervisor_io -- Core-0 live read --> AS5600 status
BLE GATT status -- direct transport state --> runtime_supervisor_io
```

BLE is a NimBLE GATT transport, not UART. RX characteristic writes are buffered
inside the BLE component and drained by `runtime_supervisor_io`; TX uses GATT
notifications. UART0 remains a separate wired development/service CLI. Neither
transport has realtime authority.

`runtime_supervisor_io` owns ingress/framing and all active-runtime wire-text
formatting. `runtime_command_parser` owns command text, aliases, numeric parsing,
and usage validation. No raw command string or line buffer crosses into Core 1.

The supervisor command mailbox is depth-limited and non-blocking. Mailbox
saturation is reported as `ERR command mailbox full`; unknown commands are
rejected on Core 0. The snapshot channel is depth one with overwrite/peek
semantics, and the first snapshot is published before supervisor ingress starts.

## Unified bounded Core-1 -> Core-0 egress

Active-runtime output no longer has separate reply/event/text paths. Core 1
publishes one of four fixed-size record classes into one FIFO so publication
order is preserved:

```text
RuntimeReply           <= 104 bytes
RuntimeStateEvent      <= 32 bytes
RuntimeTelemetryFrame  <= 128 bytes
RuntimeProfileReport   <= 152 bytes

RuntimeEgressRecord    <= 160 bytes
queue depth            = 32
publish timeout        = 0
```

Queue saturation increments one explicit egress-drop counter and never blocks the
1 kHz control task. Core 0 drains the FIFO, formats the existing protocol strings,
and sends them through the already-buffered UART/BLE transport path.

The same FIFO carries the bounded timing-profile burst (header, stages, end),
swing transition observability, ordinary command replies, asynchronous fault and
calibration events, periodic telemetry, and the parallel-control profile summary.
There is therefore no reply-only side queue that can reorder one runtime output
class relative to another.

## Read-only diagnostics

Core 0 owns the externally reachable snapshot/transport-backed read-only forms:

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
Read-only formatting happens on Core 0 using the cached snapshot.

## Structured command and asynchronous egress

The synchronous mutation-response cutover covers the active command surface:

```text
swing ownership rejection / start / abort / config / status
motor stop / stop / vq / config / calibration start
field
attitude reset
IMU calibration start / map
timing reset / timing profile status/on/off/reset
fault clear
telemetry on/off
log prepare/start/critical/stop/dump admission and results
```

Core 1 applies state/safety policy and publishes structured outcomes; Core 0
constructs command protocol text and post-command prompts.

The asynchronous cutover now also covers:

```text
newly latched FAULT events
motor-calibration completion and failure events
FOC forced-stop diagnostics
IMU gyro-calibration completion
20 ms diagnostic telemetry
parallel-control profile summary
swing transition events
```

`runtime_state.cpp` no longer constructs those asynchronous wire strings. The
telemetry producer only snapshots the values required by the existing wire
format into `RuntimeTelemetryFrame`. The parallel-profile producer likewise
publishes counters/totals; Core 0 computes the display mean and formats the line.

`motorStartFailure()` and `startGyroCalibration()` remain protocol-neutral state
helpers. Timing-profile status is still a bounded structured batch; no timing
profile `snprintf` executes in the realtime command path.

## Composition cleanup: complete

The following transitional mechanisms have been removed:

```text
raw command line crossing into Core 1
handleSupervisorCommand(event.line)
legacy swing/timing string parsers
supervisor prompt-only bookkeeping events
separate swing event queue/task
reply-only asynchronous output paths
updateEncoder/updateImu rename shims
initEncoderBus/initImuBus rename shims
runtime_control.cpp wrapper
app_main rename shim
all .cpp source inclusion
```

The architecture contract now requires:

```text
cpp_includes = 0
renaming_shims = 0
app_main owner = runtime_main.cpp
legacy app_main.cpp absent
runtime_control.cpp absent
runtime_diagnostics.cpp explicit
RuntimeCommand <= 64 bytes
unified runtime egress queue depth = 32
synchronous command formatting = Core 0
asynchronous state-event formatting = Core 0
telemetry formatting = Core 0
parallel-profile formatting = Core 0
swing transition formatting = Core 0
timing-profile formatting = Core 0
```

## Remaining A2 cleanup

The large runtime-domain boundary is now effectively cut over. Remaining work is
narrow cleanup rather than another ownership migration:

- startup-only banner/status/help and startup fatal diagnostics remain direct by
  design because they run before the realtime/supervisor boundary is active;
- TWLG binary dump transfer runs in its dedicated Core-0 task and is not a
  realtime egress path;
- four legacy typed enum/parser aliases for snapshot-owned `status`, `motor
  status`, `imu status`, and `log status` remain as compatibility fallback. The
  exact Core-0 read-only forms intercept them first; the aliases can be removed
  once command-line normalization is shared by the read-only path;
- final CI plus hardware timing validation is still required before A2 is
  considered closed.

## Target

```text
runtime_main.cpp             app entry + realtime orchestration
runtime_state.cpp/.hpp       state/control primitives
runtime_supervisor_io.*      Core-0 transport/framing/formatting
runtime_command_parser.*     Core-0 command grammar
runtime_egress.hpp           bounded Core-1 -> Core-0 publication API
runtime_reply.hpp            synchronous reply/swing/timing records
runtime_state_event.hpp      asynchronous state events
runtime_telemetry.hpp        periodic telemetry data
runtime_profile_report.hpp   parallel timing/profile data
runtime_snapshot.*           latest read-only runtime state
runtime_diagnostics.*        bounded diagnostics + Core-0 sensor status
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
- externally reachable snapshot-backed read-only formatting is supervisor-owned;
- synchronous command results and asynchronous runtime protocol events cross a
  bounded, zero-timeout Core-1 -> Core-0 structured egress path;
- periodic telemetry and profile summaries are formatted on Core 0;
- no prompt-only bookkeeping needs to cross into Core 1;
- command protocol and identification behavior remain preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green before merge.
