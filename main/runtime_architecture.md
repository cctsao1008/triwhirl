# Runtime ownership

This note is temporary refactor guidance for issue #32. It records the ownership
rules that the active firmware must satisfy while source-inclusion composition is
removed.

## Realtime domain (Core 1)

- waits on the 1 kHz GPTimer release authority;
- consumes sensor results and updates estimator/safety/control state;
- computes motor output and updates the MCPWM compare values;
- records only bounded in-memory realtime data;
- does not parse commands, format diagnostic strings, or perform flash writes.

## I/O and acquisition domain (Core 0 / supervisory tasks)

- owns AS5600 acquisition work dispatched by the realtime task;
- owns UART/BLE command transport and diagnostic output;
- owns non-realtime logger finalization and dump operations;
- publishes bounded data into the realtime domain rather than sharing timing authority.

## Transitional composition

The current active source path still contains one temporary inclusion chain:

```text
runtime_control.cpp
  -> runtime_main.cpp
     -> app_main.cpp
```

This is composition debt only. New behavior must not depend on adding more
`.cpp` inclusion, macro interception, or task-name interception. The next A2
slices remove this chain by extracting explicit shared runtime state/services,
supervisor orchestration, and startup ownership.

## Clean-code rules for A2

1. One responsibility per translation unit.
2. No new global API interception or symbol-renaming shims.
3. Preserve command protocol and measured realtime behavior during structural cuts.
4. Prefer explicit typed interfaces over anonymous cross-file state coupling.
5. Keep hardware ownership visible in names and APIs.
6. Do not mix diagnostic formatting with the hard realtime control path.
7. Delete obsolete compatibility code as soon as its replacement is validated.
