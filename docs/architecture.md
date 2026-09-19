# Repository architecture

TriWhirl is a single-target ESP32 firmware project with supporting PC-side engineering tools. The repository structure should reflect that boundary directly rather than implying unsupported MCU targets.

## Top-level layout

```text
triwhirl/
├── src/             ESP32 application and hardware/runtime integration
├── include/         project headers used by the firmware application
├── lib/             reusable, platform-independent C++ components
├── test/            native and embedded tests
├── tools/           PC-side engineering and research tools
├── docs/            design, hardware, experiments, and control documentation
├── platformio.ini   pinned ESP32 build definition
├── README.md
└── LICENSE
```

## Dependency direction

The intended dependency direction is:

```text
tools/  ---- generated/identified parameters ----> firmware
                                               
                         src/
                          |
                          v
                    lib/triwhirl_core
```

`src/` may depend on ESP32, Arduino, Wire, SimpleFOC, and board-specific definitions. Code in `lib/triwhirl_core` must remain independent of Arduino/ESP32 APIs so the same implementation can be exercised by native tests.

`tools/` runs on the development PC. It may analyze logs, identify models, run simulations, solve LMIs, and generate or verify constants. It is not part of the real-time control loop.

## Placement rules

- Put hardware integration and runtime orchestration in `src/`.
- Put project headers required by firmware integration in `include/triwhirl/`.
- Put reusable deterministic C++ logic that can be tested without the target in `lib/triwhirl_core/`.
- Put PC-only analysis, calibration, identification, synthesis, and simulation code in `tools/`.
- Do not add an `esp32/` directory solely to restate the fixed target.
- Do not add MCU portability layers without an actual second target.
- Do not add empty architectural directories before code or documentation needs them.

## Planned firmware domains

As functionality is implemented, `src/` may grow by responsibility rather than by processor target, for example:

```text
src/
├── board/
├── drivers/
├── estimation/
├── control/
├── telemetry/
├── safety/
└── main.cpp
```

These directories should be introduced only when the corresponding implementation exists.

## Control boundary

The project keeps offline design separate from embedded execution:

```text
tools/identification + tools/modeling
                |
                v
tools/synthesis/hinf       (offline LMI solve)
                |
                v
        controller constants
                |
                v
src/control/hinf           (runtime state feedback)
                |
                v
        Vq -> SimpleFOC -> motor
```

The ESP32 runtime never depends on Python or an online convex solver.

## Current development priority

Repository structure is stabilized first. The active implementation track then resumes with reaction-wheel motor bring-up, followed by sensing/estimation and robust balance control.
