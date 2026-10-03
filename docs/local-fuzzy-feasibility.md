# Historical local B/C feedback diagnostic

This note records a **historical model-quality diagnostic**, not a controller-design gate.

## Shared 120-degree control coordinate

TriWhirl has three physically equivalent upright orientations separated by 120 degrees. Runtime balance does not treat A/B/C as separate controller modes or separate plants. `upright_geometry.hpp` folds all three into the same local coordinate:

```text
global body angle
      |
      v
periodicUprightErrorRad(...)
      |
      v
theta_error in [-pi/3, pi/3)
      |
      v
pure-fuzzy attitude controller
      |
      v
target_velocity [rad/s]
```

The same fuzzy rule surface therefore operates at every physical upright orientation.

## What the historical B/C calculation actually says

The old B and C fixtures were fitted separately and remain provisional. After algebraically eliminating the historical `Vq` coordinate in favor of commanded wheel acceleration, each fixture has the form

```text
theta_dot  = theta_rate
theta_ddot = a*theta + b*theta_rate + c*wheel_rate + d*wheel_accel
wheel_dot  = wheel_accel
```

`tools/sitl/local_shared_feedback_feasibility_test.cpp` derives the necessary Hurwitz coefficient inequalities for one differentiable memoryless local feedback Jacobian and verifies a non-negative Farkas certificate showing that the two **historical numerical fits** cannot satisfy those inequalities simultaneously.

That numerical statement remains valid for the fixtures. The earlier conclusion that it must block shared fuzzy tuning does not.

A/B/C are the same 120-degree-periodic physical balance problem, so disagreement between separately fitted historical B/C models is evidence about the fits, acquisition/normalization, or unmodeled asymmetry. It is not a requirement for the production controller to stabilize two independently defined plants.

## Current engineering use

Keep the B/C certificate as a regression/diagnostic artifact only. It may help detect when old model fixtures change, but it must not:

- block pure-fuzzy rule development;
- introduce physical vertex identity into the controller;
- be interpreted as a hardware-balance impossibility result;
- force a linear/state-feedback architecture into the attitude-control path.

Current target architecture remains:

```text
periodic local theta_error
        + theta_rate
        + wheel velocity
              |
              v
      pure-fuzzy attitude control
              |
              v
      target_velocity [rad/s]
              |
              v
       SimpleFOC velocity layer
```

The first pure-fuzzy rule surface is an engineering seed for structural/SITL work only. Physical normalization scales, motor/output polarity, target-velocity limit, and final singleton values still require supported bench evidence.
