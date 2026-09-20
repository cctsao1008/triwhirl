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
Core 0                              Core 1
------                              ------
runtime_encoder_acquisition   ->    runtime_control
  AS5600 raw read                   wheel-state commit
  bounded result queue              miss/safety policy

runtime_supervisor_io         ->    runtime_control
  UART/BLE byte polling              at most one queued event / RT iteration
  line assembly                      typed mutation execution
  string parsing                     legacy string fallback (transitional)
  bounded command queue

runtime_supervisor_io         <-    runtime_snapshot
  read-only formatting               latest complete snapshot
  attitude/fault status              non-blocking overwrite publication
```

The supervisor mailbox is depth-limited and non-blocking. When full, the newest
command/event is rejected and the transport reports `ERR command mailbox full`
rather than blocking either domain.

The snapshot channel is depth one and uses overwrite/peek semantics. Core 1
publishes the latest complete snapshot without blocking; Core 0 reads a complete
copy without consuming it. The first migrated read-only commands are
`attitude status` and `fault status`, preserving their existing wire format while
moving both formatting and live-state access out of the realtime task.

Typed command migration has also started. `runtime_command.hpp` defines the
fixed-size supervisor/realtime mutation contract. `motor stop` and `swing abort`
are parsed on Core 0 into `RuntimeCommand` records and executed on Core 1 without
string interpretation. Existing swing-ownership rejection and response text are
preserved.

This remains an incremental B2 slice. Commands that are not yet migrated still
enter Core 1 through the legacy string event, so string parsing has not yet been
fully removed from realtime. The typed allow-list should grow command by command
until the legacy path can be deleted, rather than adding a second broad parser
inside realtime control.

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
runtime_encoder_acquisition.cpp/.hpp
runtime_supervisor_io.cpp/.hpp
runtime_snapshot.cpp/.hpp
```

## Acceptance

- no `.cpp` source inclusion;
- no symbol-renaming or generic ESP-IDF API interception;
- one explicit application entry path;
- one explicit realtime control task;
- UART/BLE byte polling and line assembly stay outside realtime control;
- command ingress is fixed-size, bounded, non-blocking, and explicitly rejects
  overflow;
- migrated mutating commands cross as typed `RuntimeCommand` records;
- string parsing/formatting is removed from realtime control before A2 closes;
- read-only diagnostics consume a bounded runtime snapshot rather than live
  cross-core state;
- command protocol and identification behavior preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
