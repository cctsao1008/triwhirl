# TriWhirl standup SITL

Native software-in-the-loop harnesses for TriWhirl standup/balance commissioning.
The production `triwhirl::StandupController` is compiled directly; there is no
duplicate Python/JavaScript controller.

## Two deliberately separate model layers

### 1. Local balance regression authority

```text
provisional B/C local plant
    -> [theta_error, theta_rate, wheel_rate]
    -> production StandupController
    -> target / PI / damping / saturation / slew
    -> applied Vq
    -> RK4 local plant
```

This is the existing evidence-backed regression path near upright. The built-in
B/C matrices come from existing `swing-native-04` local fits; `nominal` is their
element-wise midpoint. They remain provisional commissioning models, not a
validated digital twin.

The deterministic 10-second local balance gate and deterministic disturbance
recovery gate remain CI pass/fail tests.

### 2. Global rolling observation model

The old hand-written far-field surrogate

```text
theta_ddot = (5/3) sin(3 theta) - 1.5 theta_dot + 4 Vq
```

has been retired.

`triwhirl-standup-sitl-full` now uses geometry-derived rolling mechanics:

- ideal Reuleaux triangle support/contact from the exact intersection-of-disks geometry;
- constant-width / 120-degree periodicity emerges from geometry rather than a forced sine term;
- no-slip horizontal translation follows the instantaneous native contact point;
- COM height `h(theta)` supplies gravitational potential;
- geometry-dependent rolling inertia enters the Lagrange equation;
- the otherwise unknown effective body inertia and body Vq authority are anchored to the selected B/C local linearization at upright;
- far-field wheel self-rate/Vq terms reuse the selected local evidence;
- inside +/-2 deg the complete selected local derivative is used; a smooth residual transition ends by +/-5 deg.

Nominal outer width is currently `75 mm` from seller-level product information.
COM offset and rolling-loss parameters are not measured, so the global model is
**exploratory**. A simulated swing-up or Stable indication is an observation,
not validation authority.

## Timing

- production controller opportunity: 1 kHz;
- virtual plant RK4 step: 100 us;
- deterministic batch runs have no wall-clock scheduling or randomness;
- live WebUI runs until explicit Stop.

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

Built executables:

```text
triwhirl-standup-sitl                 local deterministic balance regression
triwhirl-standup-sitl-disturbance     local deterministic disturbance regression
triwhirl-standup-sitl-full            geometry-derived global observation
triwhirl-standup-sitl-live            continuous WebUI native process
```

## Local regression gates

```powershell
.\build\sitl\Release\triwhirl-standup-sitl.exe --self-test
```

The 10-second local balance gate requires continuous Balance, settling/stable
acquisition, bounded body error/wheel speed, bilateral correction, and no Vq
saturation.

## Geometry-derived global checks

```powershell
.\build\sitl\Release\triwhirl-standup-sitl-full.exe --self-test
```

CI checks:

- Reuleaux constant-width support geometry;
- contact remains on the ground;
- upright COM height is above the resting orientation;
- analytic support derivative matches finite difference;
- global model linearizes to the selected local gravity/body-Vq/wheel-reaction anchors at upright.

A full swing observation can be generated with:

```powershell
.\build\sitl\Release\triwhirl-standup-sitl-full.exe `
  --profile nominal `
  --duration-ms 12000 `
  --output build\sitl\full-swing.csv
```

The output intentionally reports:

```text
geometry_gate=PASS|FAIL
sequence_observed=YES|NO
validation_authority=NONE
```

There is no `--require-standup-gate`; that former gate was removed because the
far-field model does not yet have measured COM offset or rolling loss.

## WebUI

See:

```text
tools/visualization/triwhirl-sim-viewer/README.md
```

The browser does not integrate the plant. Native C++ streams body center x/y,
contact x/y, body angle, reaction-wheel state, and controller output so the
rendered rolling/contact motion is the same geometry used by the global model.

## Evidence boundary

Passing local regression gates means the production controller is stable on the
stated provisional local models for those scenarios. Passing geometry invariant
checks means the global rolling implementation is internally consistent with the
ideal Reuleaux geometry and the chosen local anchors. Neither statement proves
hardware stability.
