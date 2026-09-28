# TriWhirl Simulation Console

Local WebUI for the native standup SITL.

The browser is a display/control surface only. It does not integrate the plant,
choose a contact point, or contain controller equations.

## Scenarios

### Full swing from rest (exploratory)

Starts at the ideal Reuleaux resting orientation (`-60 deg` periodic upright
error) and runs the production `StandupController` against the geometry-derived
global rolling model until Stop.

This mode is **not a full-standup validation gate**. Unknown COM offset and
rolling losses are not measured. Reaching Balance or Stable is reported as an
observation only.

### Upright balance only

Uses the existing provisional B/C local plant directly and remains the focused
local regression/disturbance scenario.

## Architecture

```text
browser controls
   |  Run / disturbance / Stop
   v
Python HTTP/SSE bridge
   |  stdin commands / JSON-lines evidence
   v
persistent native C++ live SITL
   |
   +-> production StandupController @ 1 kHz
   +-> native plant RK4 @ 100 us
   +-> native Reuleaux support/contact + no-slip rolling pose
   +-> body/wheel disturbance impulses
```

For full mode the native sample includes:

```text
body angle
body center x/y
contact x/y
body-frame contact point
periodic upright error
reaction-wheel angle/rate
controller phase / settling / stable
Vq and wheel-target internals
```

The renderer uses those native x/y/contact values directly. It no longer keeps
the body at a fixed horizontal pivot and no longer recomputes the lowest point in
JavaScript.

## Global model boundary

The former hand-written far-field model

```text
theta_ddot = (5/3) sin(3 theta) - 1.5 theta_dot + 4 Vq
```

was retired after the resulting motion was visibly nonphysical.

The replacement derives far-field body motion from:

- exact ideal-Reuleaux support/contact geometry;
- no-slip rolling kinematics;
- COM-height gravitational potential;
- geometry-dependent rolling inertia;
- upright inertial/actuator anchors from the selected provisional B/C local fit.

The nominal width is currently 75 mm from seller-level product information.
Measured COM offset and rolling-loss parameters are still missing, therefore the
UI permanently labels global full-swing behavior as exploratory / simulation-only.

## Build

Windows Visual Studio generator:

```powershell
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl --config Release
```

On Windows the executables are normally under `build/sitl/Release/` or `Debug/`.

## Launch

```powershell
python tools/visualization/triwhirl-sim-viewer/serve_live.py
```

Open:

```text
http://127.0.0.1:8000/tools/visualization/triwhirl-sim-viewer/
```

Press **Run live**. There is no simulation-duration limit; the native process
continues until **Stop** is pressed or the browser/server disconnects.

The plot shows only the most recent 10 seconds. That is a display window, not a
simulation limit.

## Interactive disturbances

While a live session is running:

- **Body push Δθdot** adds a signed body angular-rate impulse to the current native state.
- **Wheel kick Δomega** adds a signed reaction-wheel-rate impulse to the current native state.

The already-running native process applies the disturbance at its current
simulation time.

## CI

CI keeps separate responsibilities:

- deterministic local balance regression gate;
- deterministic local disturbance-recovery gate;
- geometry-derived Reuleaux invariant checks;
- exploratory full-swing evidence generation with `validation_authority=NONE`;
- WebUI bridge test proving rest -> rolling swing/reversal -> disturbance -> Stop.

There is intentionally no automated full-standup PASS gate until the missing
global physical parameters are measured or otherwise justified.

## Evidence boundary

The console permanently displays:

```text
SIMULATION ONLY
NO PHYSICAL AUTHORITY
```

Local B/C behavior is provisional commissioning evidence. Global full-swing
behavior is a geometry-consistent exploratory model. Neither authorizes hardware
by itself.
