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

## Target-velocity motor-servo contract

The target architecture does not expose `Vq` to the attitude controller. Before
tuning full-fuzzy control, SITL therefore has a separate mechanical-domain
contract for the future actuator boundary:

```text
target_velocity [rad/s]
        |
        v
provisional velocity-servo surrogate
        |
        v
wheel_accel_command [rad/s^2]
```

`velocity_servo_model.hpp` implements a bounded first-order surrogate:

```text
wheel_accel_command = clamp((target_velocity - wheel_velocity) / tau,
                            -acceleration_limit,
                            +acceleration_limit)
```

Its configuration is invalid by default. The round values used by the contract
test are simulation fixtures only: they are **not** SimpleFOC PI/LPF settings,
not hardware commissioning data, and not vendor-derived parameters.

This contract is intentionally separate from the existing Vq-input B/C
commissioning SITL. It proves units, bounds, symmetry, saturation behavior and
fail-closed numeric handling only. It does not identify the real motor servo,
change current local-model authority, or validate global swing-up.

## Local actuator-coordinate transform

The provisional B/C local models were identified with `Vq` as their input:

```text
x_dot = A x + B * Vq
```

The target-velocity servo contract instead emits commanded wheel acceleration.
Those quantities are not interchangeable. `local_linear_plant.hpp` therefore
adds an algebraic simulation-only coordinate transform around the same local
linear model.

For the identified wheel-rate row,

```text
wheel_accel = A_w x + B_w * Vq
```

the transformed coordinate solves the latent old input as

```text
Vq_equiv = (wheel_accel_command - A_w x) / B_w
```

and uses that value only inside the local-model algebra. `Vq_equiv` is not a
production actuator command and is never an output of the fuzzy controller.
The transformed derivative uses

```text
theta dynamics <- original body rows evaluated at Vq_equiv
wheel_dot      <- wheel_accel_command
```

The deterministic contract constructs `wheel_accel_command` from the original
Vq derivative over B, C, and nominal fixtures and verifies that the transformed
state derivative reproduces the original derivative to numerical tolerance.
It also checks the full simulation boundary

```text
target_velocity
    -> provisional velocity servo
    -> wheel_accel_command
    -> transformed local plant derivative
```

for finite, dimensionally explicit behavior. This is a coordinate change of a
**provisional near-upright model**, not an upgrade of plant authority and not a
claim that the real SimpleFOC motor path is an ideal acceleration source.

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
triwhirl-standup-sitl                         local deterministic balance regression
triwhirl-standup-sitl-disturbance             local deterministic disturbance regression
triwhirl-standup-sitl-full                    geometry-derived global observation
triwhirl-standup-sitl-live                    continuous WebUI native process
triwhirl-velocity-servo-contract              target-velocity mechanical-boundary contract
triwhirl-local-accel-coordinate-contract      local Vq/wheel-acceleration parity contract
```

## Local regression gates

```powershell
.\build\sitl\Release\triwhirl-standup-sitl.exe --self-test
```

The 10-second local balance gate requires continuous Balance, settling/stable
acquisition, bounded body error/wheel speed, bilateral correction, and no Vq
saturation.

The independent target-velocity boundary contract can be run with:

```powershell
.\build\sitl\Release\triwhirl-velocity-servo-contract.exe
```

It gates invalid configuration, zero-error behavior, sign symmetry, target and
acceleration clamping, monotonic unsaturated response, and non-finite input
rejection without changing the existing commissioning controller.

The local actuator-coordinate parity contract can be run with:

```powershell
.\build\sitl\Release\triwhirl-local-accel-coordinate-contract.exe
```

It verifies B/C/nominal Vq-to-wheel-acceleration derivative parity, rejects
non-invertible/non-finite fixtures, and checks the velocity-servo-to-local-plant
simulation boundary. It does not assert closed-loop fuzzy stability.

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
stated provisional local models for those scenarios. Passing the target-velocity
servo contract means only that the future mechanical command boundary is
finite, bounded and dimensionally explicit for the stated simulation fixture.
Passing the local actuator-coordinate contract means the new wheel-acceleration
coordinate is algebraically equivalent to the old Vq coordinate for the tested
provisional local fixtures. Passing geometry invariant checks means the global
rolling implementation is internally consistent with the ideal Reuleaux
geometry and the chosen local anchors. None of these statements proves hardware
stability or identifies the real SimpleFOC servo dynamics.
