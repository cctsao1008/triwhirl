# Local fuzzy-balance feasibility gate

This note records a **model-feasibility boundary**, not a controller-design conclusion.

## Scope

TriWhirl's target near-upright architecture is:

```text
mechanical state
    -> fuzzy attitude controller
    -> target_velocity [rad/s]
    -> first-order SITL velocity-servo abstraction
    -> wheel_accel_command
    -> provisional local mechanical plant
```

The current local B/C fixtures are historical, provisional models. New plant identification is vertex-agnostic, so B/C disagreement is treated as model uncertainty/evidence quality rather than as permission to add a physical-vertex controller mode.

## Wheel-acceleration local form

Eliminating the historical Vq coordinate through the existing local actuator-coordinate transform gives

```text
theta_dot  = theta_rate
theta_ddot = a*theta + b*theta_rate + c*wheel_rate + d*wheel_accel
wheel_dot  = wheel_accel
```

For the current repository coefficients, the derived rows are approximately:

| fixture | a | b | c | d |
|---|---:|---:|---:|---:|
| B | 152.3662139 | 3.8855771 | -0.6894016 | 0.02815043 |
| C | 199.8710830 | 5.7227181 | -1.0281588 | 0.14072276 |

These numbers remain provisional plant evidence. They are not hardware truth.

## Necessary local stability test

For a shared static local acceleration-feedback Jacobian

```text
wheel_accel = f1*theta + f2*theta_rate + f3*wheel_rate
```

the closed-loop characteristic polynomial is

```text
lambda^3 + alpha2*lambda^2 + alpha1*lambda + alpha0
```

with

```text
alpha2 = -(b + d*f2 + f3)
alpha1 = -a + b*f3 - c*f2 - d*f1
alpha0 =  a*f3 - c*f1
```

A Hurwitz cubic necessarily requires

```text
alpha2 > 0
alpha1 > 0
alpha0 > 0
```

before the additional cubic product inequality is even considered. These necessary conditions are linear inequalities in `(f1, f2, f3)`.

## Current B/C infeasibility certificate

`tools/sitl/local_shared_feedback_feasibility_test.cpp` derives those inequalities directly from `provisionalLocalB()` and `provisionalLocalC()` and verifies a non-negative Farkas certificate.

For the current coefficients, one normalized certificate has non-zero multipliers on:

```text
B.alpha2 : 1.0
B.alpha1 : 0.179405739172411
C.alpha2 : 0.678867933361362
C.alpha0 : 0.004912031758055
```

The CI test recomputes the weighted coefficient residual from the live fixtures. At the current revision it is numerically zero to floating-point tolerance, while the weighted right-hand side is about:

```text
-35.1059201143
```

Therefore, if all necessary inequalities were simultaneously true, their non-negative weighted sum would require

```text
0 < -35.1059201143
```

which is impossible.

## Relation to `target_velocity`

Around an unsaturated point, the current SITL velocity-servo abstraction is

```text
wheel_accel = (target_velocity - wheel_velocity) / tau
```

for finite positive `tau`. Its local target-velocity Jacobian `K` and acceleration-feedback Jacobian `F` are related bijectively:

```text
F = {K0/tau, K1/tau, (K2 - 1)/tau}
K = {tau*F0, tau*F1, 1 + tau*F2}
```

So changing the local command coordinate does not remove this particular incompatibility between the current B/C fixtures.

## What this means

The supported conclusion is deliberately narrow:

> The current provisional historical B/C pair, under the current first-order target-velocity servo abstraction, cannot share one differentiable memoryless local controller Jacobian that makes both modeled cubics Hurwitz.

This is a **plant-evidence blocker before shared fuzzy tuning**. It is not evidence that fuzzy control is impossible, that SimpleFOC is unsuitable, or that the physical TriWhirl cannot balance.

A nonsmooth, dynamic, adaptive, or differently modeled controller is outside this certificate. More importantly, the historical B/C fits themselves may not represent a coherent supported uncertainty set.

## Engineering consequence

Do not hand-tune 125 fuzzy singletons to hide this disagreement. Do not add physical vertex identity merely to rescue the historical fits. Keep hardware balance blocked.

The next useful step is to obtain/reconcile new vertex-agnostic active local identification and holdout replay evidence. Once the local plant set is coherent enough to support a common near-upright control problem, fuzzy rule-surface tuning can resume against that evidence.
