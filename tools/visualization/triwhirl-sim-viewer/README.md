# TriWhirl Simulation Console

Local WebUI for the native standup SITL.

The console has two deliberately separate roles:

1. **Interactive live simulation** — starts a persistent native C++ SITL process and runs until the user presses **Stop**. Disturbance buttons modify the currently running native plant state.
2. **Deterministic CI gates** — keep the existing finite 10-second balance and disturbance-recovery runs as repeatable regression tests.

The browser does not contain controller equations or a plant integrator.

## Architecture

Interactive mode:

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
   +-> provisional local plant RK4 @ 100 us
   +-> body/wheel disturbance impulses
```

The browser is a display/control surface only. Plant integration and controller execution stay in native C++.

The deterministic batch executables remain separate so CI can retain fixed-duration, reproducible pass/fail gates.

## Build

From the repository root on Windows with the Visual Studio CMake generator:

```powershell
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl --config Release
```

This builds:

```text
triwhirl-standup-sitl                 deterministic balance gate
triwhirl-standup-sitl-disturbance     deterministic scheduled-disturbance gate
triwhirl-standup-sitl-live            continuous interactive native session
```

On Windows they are normally under `build/sitl/Release/` or `Debug/`.

## Launch

```powershell
python tools/visualization/triwhirl-sim-viewer/serve_live.py
```

Then open:

```text
http://127.0.0.1:8000/tools/visualization/triwhirl-sim-viewer/
```

Press **Run live**. There is no simulation-duration limit in the WebUI. The native process continues until **Stop** is pressed or the browser/server disconnects.

The rolling plot still shows the most recent 10 seconds; that is only the display window, not a simulation limit.

## Interactive disturbances

While a live session is running:

- **Body push Δθ̇** adds a signed body angular-rate impulse to the current native state.
- **Wheel kick Δω** adds a signed reaction-wheel-rate impulse to the current native state.

The disturbance is applied by the already-running native process at its current simulation time. The browser does not redraw a fake disturbance and the backend does not rewind/recompute the trajectory.

## What is rendered

- actual local body error from the virtual plant;
- reaction-wheel angle and speed;
- production controller phase, settling and stable flags;
- filtered body rate;
- wheel target / velocity error / PI integral;
- unclamped and applied Vq;
- rolling 10-second body-error and Vq history;
- disturbance markers;
- selected provisional plant profile;
- current live status (`RUNNING`, `STABLE`, `OUT OF BALANCE`, or `STOPPED`).

The body drawing uses the same sign as the SITL local angle. The browser does not flip a sign or exaggerate body motion to make the animation look stable.

## Deterministic 10-second CI gates

Removing the WebUI time limit does **not** remove the automated regression gates. CI still executes finite 10-second native runs because fixed-duration evidence is reproducible and suitable for pass/fail automation.

The balance gate currently checks items such as continuous Balance phase, settling/stable acquisition, bounded body error and wheel speed, Vq limits, and bilateral correction. A separate deterministic disturbance run injects scheduled impulses and verifies recovery.

These are regression tests, not the interactive UI lifetime.

## Evidence boundary

The B/C local models remain **provisional commissioning evidence**. The nominal profile is their element-wise midpoint. B and C can be selected to expose model sensitivity, but behavior in these models is not hardware authorization by itself.

Therefore the console permanently displays:

```text
SIMULATION ONLY
NO PHYSICAL AUTHORITY
```
