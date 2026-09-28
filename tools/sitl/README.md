# TriWhirl standup SITL

Deterministic native software-in-the-loop harness for standup/balance
commissioning. It compiles the production `triwhirl::StandupController`
directly; there is no duplicate Python/JavaScript controller.

## Two simulation layers

TriWhirl deliberately keeps two plant scopes instead of pretending one model is
valid everywhere.

### Local balance SITL

```text
provisional identified local plant
    -> [theta_error, theta_rate, wheel_rate]
    -> production StandupController
    -> target / PI / damping / saturation / slew
    -> applied Vq
    -> RK4 plant
```

The B/C matrices come from the existing `swing-native-04` local fits. Their input
column is sign-normalized to the current Vq coordinate. `nominal` is the
component-wise B/C midpoint. These remain commissioning models, not a validated
digital twin.

### Full standup SITL

The full-standup runner starts from a resting orientation near `-59 deg` periodic
upright error and exercises the actual production sequence:

```text
resting face
    -> SwingHigh / SwingLow energy pump
    -> +/-9 deg capture
    -> Balance
    -> settling latch
    -> Stable
```

The missing far-field physics is represented by an explicit 120-degree periodic
surrogate:

```text
body gravity-like term = (k/3) * sin(3 * theta_error)
```

with finite body/wheel damping and Vq authority. This gives stable resting
orientations at +/-60 deg and unstable upright vertices every 120 deg.

The global surrogate is **not identified contact mechanics**. Near upright it is
smoothly stitched back to the existing local plant:

- within +/-2 deg: 100% identified local B/C plant;
- from 2 to 5 deg: smooth blend;
- beyond +/-5 deg: periodic far-field surrogate.

Its purpose is to validate the controller/state-machine sequence and provide a
visible pre-hardware swing-up test, not to claim a globally validated Reuleaux
triangle digital twin.

## Shared production configuration

The ESP32 runtime and all SITL runners obtain commissioning overrides from
`triwhirl::makeStandupCommissioningConfig()` so simulation and firmware cannot
silently use different gains.

## Timing

- production controller opportunity: 1 kHz;
- virtual plant RK4 step: 100 us;
- no randomness in deterministic batch gates;
- virtual timestamp starts at 1 s to avoid conflating timestamp zero with the
  controller's uninitialized-time sentinel.

## Build

Windows Visual Studio generator:

```powershell
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl --config Release
```

Linux/macOS:

```bash
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl
```

The build provides:

```text
triwhirl-standup-sitl               local balance/invariant gates
triwhirl-standup-sitl-full          deterministic rest-to-upright gate
triwhirl-standup-sitl-disturbance   scheduled local disturbance gate
triwhirl-standup-sitl-live          continuous WebUI native session
```

## Local balance gate

Run the native invariant and 10-second balance suite:

```powershell
.\build\sitl\Release\triwhirl-standup-sitl.exe --self-test
```

The 10-second local gate checks continuous Balance operation, settling/stable
acquisition, upright crossing, bilateral Vq reversal, body/wheel bounds and Vq
saturation.

## Full standup gate

Run the complete production standup sequence from rest:

```powershell
.\build\sitl\Release\triwhirl-standup-sitl-full.exe `
  --profile nominal `
  --duration-ms 12000 `
  --require-standup-gate `
  --output build\sitl\full-standup.csv
```

The full gate requires evidence of:

- both `swing_high` and `swing_low`;
- at least one body-rate reversal during energy pumping;
- delayed entry into Balance rather than starting upright;
- settling and `stable=true` acquisition;
- ending in Balance/Stable;
- final two-second periodic error below 0.5 deg;
- bounded wheel speed and Vq with no Vq saturation.

CI keeps both the local-balance gate and this full rest-to-upright gate.

## WebUI

For the continuous browser console, see:

```text
tools/visualization/triwhirl-sim-viewer/README.md
```

The default WebUI scenario is **Full standup from rest**. The browser remains a
display/control surface only; native C++ owns controller execution and plant
integration.

## Evidence boundary

Passing the full-standup gate proves the production controller can execute the
expected swing/capture/settling sequence on the explicitly stated hybrid
commissioning model. It does **not** validate the real global rolling/contact
physics or authorize hardware by itself. The identified evidence is strongest
near upright; the far-field model remains a transparent surrogate until better
physical or measured evidence exists.
