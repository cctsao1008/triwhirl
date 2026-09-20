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

Encoder acquisition is already isolated as a normal compiled service:

```text
runtime_control.cpp
  -> runtime_encoder_acquisition.hpp
runtime_encoder_acquisition.cpp
```

The acquisition worker owns its Core-0 queue/task mechanics and raw AS5600 read
callback. The realtime control path retains ownership of wheel-state commit,
consecutive-miss policy, and safety-visible sample validity. Acquisition stats
are updated from the control-core side so diagnostics do not introduce a new
cross-core shared-counter race.

## Target

```text
runtime_startup.cpp
runtime_control.cpp
runtime_supervisor.cpp
runtime_state.cpp/.hpp
runtime_encoder_acquisition.cpp/.hpp
runtime_platform.cpp/.hpp
runtime_release.cpp/.hpp
```

## Acceptance

- no `.cpp` source inclusion;
- no symbol-renaming or generic ESP-IDF API interception;
- one explicit application entry path;
- one explicit realtime control task;
- sensor acquisition mechanics separated from control-state ownership;
- command protocol and identification behavior preserved;
- realtime ownership remains on ESP32;
- CI and hardware timing validation remain green after each structural slice.
