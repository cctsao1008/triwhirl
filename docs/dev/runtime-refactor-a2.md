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
  UART/BLE byte polling              one bounded input event / RT iteration
  line assembly                      command execution (transitional)
  fixed-size input queue
```

The supervisor mailbox is depth-limited and non-blocking. When full, the newest
command/event is rejected and the transport reports `ERR command mailbox full`
rather than blocking either domain.

The supervisor I/O extraction is intentionally a B1 step. UART/BLE reads and
line assembly no longer execute in the 1 kHz control task, but command string
interpretation still does. The next supervisory slice replaces that remaining
string parsing with typed commands and publishes read-only runtime snapshots
back to the supervisor domain.

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
runtime_encoder_acquisition.cpp/.hpp
runtime_supervisor_io.cpp/.hpp
```

## Acceptance

- no `.cpp` source inclusion;
- no symbol-renaming or generic ESP-IDF API interception;
- one explicit application entry path;
- one explicit realtime control task;
- UART/BLE byte polling and line assembly stay outside realtime control;
- command ingress is fixed-size, bounded, non-blocking, and explicitly rejects
  overflow;
- string parsing/formatting is removed from realtime control before A2 closes;
- read-only diagnostics consume a bounded runtime snapshot rather than live
  cross-core state;
- command protocol and identification behavior preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
