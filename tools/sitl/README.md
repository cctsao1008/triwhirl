# TriWhirl standup SITL

This directory contains a deterministic native software-in-the-loop harness for
standup-controller commissioning. It follows the semantic-path idea used by the
`rotary-inverted-pendulum` SITL: run production control code against an explicit
virtual plant instead of maintaining a second controller implementation.

## Scope

The first version is deliberately narrow. It is a pre-hardware gate for:

- controller sign and bilateral symmetry;
- body-rate damping polarity;
- reaction-wheel momentum feedback;
- swing/capture/settling/fall transitions;
- PI, target saturation, Vq saturation, and slew behavior;
- short-horizon near-upright closed-loop causality.

It is **not** a validated digital twin and does not replace hardware traces.

The executable compiles the production `triwhirl::StandupController` directly
and obtains the commissioning overrides from
`triwhirl::makeStandupCommissioningConfig()`, the same factory used by the ESP32
runtime.

## Provisional plant

The local coordinate is the production balance coordinate:

```text
x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
u = Vq_v
x_dot = A x + B u
```

The built-in B/C matrices come from the existing `swing-native-04` local fits.
Those fits remain provisional because the coarse swing-up portion of that run
included human assistance. The SITL intentionally sets the affine fit bias to
zero and treats the model as equilibrium-centered commissioning evidence.

B:

```text
A = [[   0.000,  1.000,  0.000],
     [ 149.234,  4.205, -0.871],
     [-111.267, 11.347, -6.451]]
B = [0.000, 5.099, 181.134]^T
```

C:

```text
A = [[   0.000,  1.000,  0.000],
     [ 155.128,  8.850, -2.329],
     [-317.952, 22.223, -9.244]]
B = [0.000, 31.421, 223.283]^T
```

`nominal` is the element-wise B/C midpoint.

### Vq coordinate normalization

`swing-native-04` was identified using the earlier actuator/sensor-direction
coordinate, while the current motor-direction preflight establishes the present
software convention as positive Vq producing positive wheel velocity. The old
fit's input column is therefore sign-normalized for this SITL while `A` is left
unchanged. This is an explicit provisional coordinate transform, not a claim
that the historical fit has been revalidated on current hardware.

## Deterministic timing

- production controller opportunity: 1 kHz;
- virtual plant RK4 step: 100 us;
- no wall-clock scheduling or randomness;
- virtual timestamp starts at 1 s to avoid conflating timestamp zero with the
  controller's uninitialized-time sentinel.

## Build and run

Linux/macOS:

```bash
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl
build/sitl/triwhirl-standup-sitl --self-test
```

Example evidence trace:

```bash
build/sitl/triwhirl-standup-sitl \
  --scenario near-upright-positive \
  --profile nominal \
  --duration-ms 200 \
  --output /tmp/triwhirl-standup-sitl.csv
```

Profiles are `nominal`, `B`, and `C`. The evidence CSV records true virtual
state plus the production-controller phase, settling/stable flags, filtered
rate, wheel target, velocity error, PI integral, unclamped/target/applied Vq,
and saturation flags.

## Self-test contract

`--self-test` fails if any of these regressions occurs:

- positive/negative angle no longer produces mirrored restoring actuation;
- wheel momentum flips restoring authority near upright;
- body-rate damping becomes anti-damping on either side;
- a reversal around 8 degrees falsely latches settling;
- a true zero crossing fails to latch settling;
- settled correction stops at 30 degrees or does not release beyond 55 degrees;
- the 1-second `stable` qualification stops active correction;
- the short nominal near-upright trajectories lose bilateral symmetry, fail to
  cross upright, or immediately run away.

Passing this harness means the controller satisfies these commissioning
contracts on the stated provisional model. It does **not** certify long-term
balance stability, plant fidelity, or safe hardware operation.
