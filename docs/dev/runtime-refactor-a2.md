# Runtime refactor A2

Issue: #32

A2 removes runtime source-inclusion composition while preserving the validated
control behavior and command wire protocol.

## Current active composition

```text
runtime_main.cpp        explicit app entry + Core-1 realtime runtime
runtime_state.cpp/.hpp  shared runtime state and bring-up helpers
```

The historical composition chain is gone:

```text
runtime_control.cpp -> runtime_main.cpp -> app_main.cpp
```

`runtime_control.cpp` and legacy `app_main.cpp` are no longer present. There are
no `.cpp` source inclusions and no symbol-renaming shims.

## Explicit runtime boundaries

```text
Core 0                                      Core 1
------                                      ------
runtime_encoder_acquisition           ->    runtime_main
  AS5600 raw read                           wheel-state commit
  bounded result queue                      miss/safety policy

runtime_supervisor_io
  UART0 wired development/service CLI
  NimBLE GATT RX-characteristic ingress
  line assembly
       |
       v
runtime_command_parser                ->    runtime_main
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

BLE is a NimBLE GATT transport, not UART. RX characteristic writes are buffered
inside the BLE component and drained by `runtime_supervisor_io`; TX uses GATT
notifications. UART0 remains a separate wired development/service CLI. Neither
transport has realtime authority.

`runtime_supervisor_io` owns ingress/framing while `runtime_command_parser` owns
command text, aliases, numeric parsing, and usage validation. No raw command
string or line buffer crosses into Core 1.

The supervisor mailbox is depth-limited and non-blocking. Mailbox saturation is
reported as `ERR command mailbox full`; unknown commands are rejected on Core 0.
The snapshot channel is depth one with overwrite/peek semantics, and the first
snapshot is published before supervisor ingress starts.

`runtime_command.hpp` represents the remaining command surface that requires
realtime-owned state. Motor, IMU, swing, timing-profile, logger, and attitude
mutations arrive on Core 1 only as fixed-size typed records. Original aliases,
usage errors, and swing ownership behavior are preserved.

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
```

## Remaining A2 debt

Composition is clean, but A2 still has two ownership problems to finish before
closure:

- read-only aggregate `status` still refreshes AS5600 health and `imu status`
  still performs MPU6050 WHO_AM_I diagnostic I2C from realtime-owned state;
- command responses, telemetry, swing events, and some status formatting still
  originate from Core 1, so supervisor-owned single-writer protocol egress is
  not complete.

The next slices should move diagnostic acquisition/caching out of the realtime
deadline and add a bounded Core-1 -> Core-0 result/event path for formatting and
transport output. No blocking output or mutex should be introduced into Core 1.

## Target

```text
runtime_main.cpp          app entry + realtime orchestration
runtime_state.cpp/.hpp    state/control primitives
runtime_supervisor_io.*   Core-0 transport/framing/output
runtime_command_parser.*  Core-0 command grammar
runtime_snapshot.*        latest read-only runtime state
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
- no blocking diagnostic I2C is triggered by read-only supervisor commands in
  the realtime deadline;
- protocol formatting/egress is supervisor-owned through bounded non-blocking
  Core-1 -> Core-0 records;
- command protocol and identification behavior remain preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
