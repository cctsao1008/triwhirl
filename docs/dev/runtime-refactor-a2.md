# Runtime refactor A2

Issue: #32

A2 removes runtime source-inclusion composition while preserving the validated
control behavior and command wire protocol.

## Current active composition

```text
runtime_main.cpp         explicit app entry + Core-1 realtime orchestration
runtime_state.cpp/.hpp   shared runtime state and bring-up/control helpers
runtime_diagnostics.*    bounded diagnostic snapshot enrichment + Core-0 AS5600 health
runtime_reply.hpp        fixed-size Core-1 -> Core-0 reply/event contract
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
  mutation grammar                               at most one queued command / RT iteration
  numeric parsing                                typed command execution
  usage errors                                   no raw command strings
  fixed-size RuntimeCommand

runtime_supervisor_io                      <-    runtime_snapshot
  read-only formatting                           latest complete snapshot
  status / motor status                          state copied on Core 1
  imu / log / attitude / fault / timing          non-blocking overwrite publication
  telemetry query

runtime_supervisor_io                      <-    RuntimeReply queue
  synchronous result formatting                  bounded, non-blocking Core-1 publish
  swing status/events                            fixed-size structured records
  timing-profile status batch
  post-command prompt

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

The supervisor command mailbox is depth-limited and non-blocking. Mailbox
saturation is reported as `ERR command mailbox full`; unknown commands are
rejected on Core 0. The snapshot channel is depth one with overwrite/peek
semantics, and the first snapshot is published before supervisor ingress starts.

`RuntimeCommand` is a trivially-copyable fixed-size mailbox record bounded to 64
bytes. `RuntimeReply` is also trivially copyable and bounded to 104 bytes. The
reply queue is depth 24 and uses zero-timeout publication from Core 1. The larger
queue is intentional: a complete timing-profile report is a bounded burst of
header/stage/end records. Reply saturation increments an explicit drop counter
rather than blocking realtime.

## Read-only diagnostics moved out of realtime command execution

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
Read-only formatting then happens on Core 0 using the cached snapshot.

The existing wire formats for aggregate status, IMU status, and log status are
preserved.

## Synchronous command egress moved to Core 0

The synchronous mutation-response cutover is now complete for the active command
surface. Core 1 applies state/safety policy and publishes a structured result;
Core 0 constructs the protocol text and owns the post-command prompt.

Covered results include:

```text
swing ownership rejection
motor stop / stop
swing abort / start / config
swing status
timing reset
timing profile status / on / off / reset
fault clear
telemetry on / off
motor vq / config / calibration start
field
attitude reset
imu calibration start / map
log prepare / start / critical on/off / stop / dump admission errors
```

`motorStartFailure()` and `startGyroCalibration()` are protocol-neutral state
helpers. They return structured state/admission information instead of printing
command responses themselves.

Swing transition observability also uses the same bounded egress queue. The old
separate `SwingEvent` queue/task has been removed. Core 1 publishes a
`kSwingTransitionEvent` record; Core 0 formats the existing
`event,swing_id,...` wire line. `swing_event_drops` remains as the experiment
specific drop counter when structured event publication fails.

Timing-profile status is emitted as a bounded structured batch:

```text
RuntimeReply::kTimingProfileHeader
8 realtime stage records
encoder raw/status I2C stage records
MPU I2C/decode stage records
RuntimeReply::kTimingProfileEnd
```

Core 0 recreates the existing `timing_profile...` wire format. No timing-profile
`snprintf` work is done in the realtime command path.

Supervisor bookkeeping events used only to make Core 1 print a prompt have also
been removed. Core 0 handles prompts directly for read-only commands, parser
errors, mailbox rejection, and structured command replies. Prompt suppression
uses bounded runtime state for telemetry, binary dump, and swing-active state.

## Composition cleanup: complete

The following transitional mechanisms have been removed:

```text
raw command line crossing into Core 1
handleSupervisorCommand(event.line)
legacy swing/timing string parsers
supervisor prompt-only bookkeeping events
separate swing event queue/task
updateEncoder/updateImu rename shims
initEncoderBus/initImuBus rename shims
runtime_control.cpp wrapper
app_main rename shim
all .cpp source inclusion
```

`runtime_state.cpp/.hpp` explicitly owns the state and helpers that were
previously visible only because `app_main.cpp` was textually included. CMake
compiles every runtime service as a normal translation unit. The architecture
contract now requires:

```text
cpp_includes = 0
renaming_shims = 0
app_main owner = runtime_main.cpp
legacy app_main.cpp absent
runtime_control.cpp absent
runtime_diagnostics.cpp explicit
RuntimeCommand <= 64 bytes
RuntimeReply <= 104 bytes
reply queue depth = 24
synchronous command formatting = Core 0
swing transition formatting = Core 0
timing-profile command formatting = Core 0
```

## Remaining A2 debt

The remaining outbound debt is now asynchronous rather than synchronous command
handling:

- periodic 20 ms telemetry is still formatted directly in the Core-1 realtime
  path;
- safety-fault text is still emitted by the state/control helper that trips the
  latch;
- motor-calibration completion/failure and IMU gyro-calibration completion still
  originate as Core-1 text;
- the parallel-control profile summary still formats text on Core 1 when timing
  profiling is disabled;
- startup-only banner/status/help formatting remains direct by design because it
  executes before the realtime task/supervisor boundary is active;
- TWLG binary dump transfer runs in its dedicated Core-0 task; its bulk binary
  transfer is not a realtime egress path.

The next large slice should convert the remaining asynchronous runtime events to
structured Core-1 -> Core-0 records and move periodic telemetry formatting to the
supervisor domain, without introducing blocking output, heap ownership, or an
output mutex in Core 1.

Four legacy typed enum/parser aliases for snapshot-owned `status`, `motor
status`, `imu status`, and `log status` remain unreachable because the Core-0
read-only handler intercepts them first. Their deletion is cleanup, not a
functional boundary dependency, and can happen after the current egress cutover
is stable.

## Target

```text
runtime_main.cpp          app entry + realtime orchestration
runtime_state.cpp/.hpp    state/control primitives
runtime_supervisor_io.*   Core-0 transport/framing/formatting
runtime_command_parser.*  Core-0 mutation grammar
runtime_reply.hpp         structured Core-1 -> Core-0 outcomes/events
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
- externally reachable snapshot-backed read-only status formatting is
  supervisor-owned;
- synchronous command-result, swing-status/event, and timing-profile formatting
  cross the bounded non-blocking Core-1 -> Core-0 `RuntimeReply` path;
- no prompt-only bookkeeping needs to cross into Core 1;
- remaining asynchronous protocol formatting is migrated before A2 closes;
- command protocol and identification behavior remain preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
