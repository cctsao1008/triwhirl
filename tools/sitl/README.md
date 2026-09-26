# TriWhirl standup SITL

Deterministic native software-in-the-loop harness for standup/balance
commissioning. It compiles the production `triwhirl::StandupController`
directly; there is no duplicate Python/JavaScript controller.

## Semantic path

```text
provisional local plant
    -> [theta_error, theta_rate, wheel_rate]
    -> production StandupController
    -> target / PI / damping / saturation / slew
    -> applied Vq
    -> RK4 plant
```

The ESP32 runtime and SITL both obtain commissioning overrides from
`triwhirl::makeStandupCommissioningConfig()` so simulation and firmware cannot
silently use different gains.

## Timing

- production controller opportunity: 1 kHz;
- virtual plant RK4 step: 100 us;
- no wall-clock scheduling or randomness;
- virtual timestamp starts at 1 s to avoid conflating timestamp zero with the
  controller's uninitialized-time sentinel.

## Provisional plant profiles

State and input:

```text
x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
u = Vq_v
x_dot = A x + B u
```

The built-in B/C matrices come from existing `swing-native-04` local fits. The
fit input column is sign-normalized to the current Vq coordinate. These are
commissioning models, not a validated digital twin.

`nominal` is the element-wise B/C midpoint. B and C remain selectable so model
sensitivity is visible instead of hidden.

## Build and test

Windows Visual Studio generator:

```powershell
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl --config Release
.\build\sitl\Release\triwhirl-standup-sitl.exe --self-test
```

Linux/macOS:

```bash
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl
build/sitl/triwhirl-standup-sitl --self-test
```

`--self-test` includes both controller invariants and the 10-second nominal
closed-loop balance gate.

## Long-run gate

The hardware precondition is no longer the old 200 ms smoke check. A nominal
`balance-demo` run must complete at least 10 s and satisfy the native gate:

- remain in Balance;
- acquire settling and observe `stable=true`;
- cross upright and reverse applied Vq;
- max body error < 5 deg;
- final 2 s max body error < 0.5 deg;
- max wheel speed < 20 rad/s;
- applied Vq within 4 V with no Vq saturation.

Run it directly:

```powershell
.\build\sitl\Release\triwhirl-standup-sitl.exe `
  --scenario balance-demo `
  --profile nominal `
  --duration-ms 10000 `
  --require-balance-gate `
  --output build\sitl\balance-demo.csv
```

Available scenarios:

```text
near-upright-positive   +1 deg, -0.2 rad/s
near-upright-negative   -1 deg, +0.2 rad/s
balance-demo            +3 deg, -0.5 rad/s
```

Profiles: `nominal`, `B`, `C`.

The CSV includes true virtual body/wheel state, integrated wheel angle,
controller phase, settling/stable flags, target, velocity error, PI integral,
unclamped/target/applied Vq, and saturation flags.

## WebUI

For the browser-visible gate, see:

```text
tools/visualization/triwhirl-sim-viewer/README.md
```

The browser is display-only; native C++ remains authoritative for both dynamics
and pass/fail.

## Evidence boundary

Passing the nominal 10-second gate means the production controller is
closed-loop stable on the stated provisional nominal model for that scenario.
It does **not** prove hardware stability or resolve disagreement between the
provisional B/C models. Hardware remains a separate explicit decision.
