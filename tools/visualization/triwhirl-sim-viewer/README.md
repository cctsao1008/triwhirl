# TriWhirl Simulation Console

Local read-only WebUI for the deterministic native standup SITL.

The purpose of this console is to make the pre-hardware gate visible: the
Reuleaux-triangle body and reaction wheel must remain balanced in the same native
SITL that runs the production `StandupController`. The browser does not contain
controller equations or a plant integrator.

## Architecture

```text
production StandupController @ 1 kHz
        -> native C++ provisional local plant
           RK4 @ 100 us
        -> fresh CSV evidence
        -> Python SSE bridge (display downsample only)
        -> browser Canvas renderer <= 60 fps
```

This mirrors the authority boundary used by the Rotary simulation console:
JavaScript is a display projection only. It may visualize evidence; it may not
change the physics or control law.

## Build

From the repository root on Windows with the Visual Studio CMake generator:

```powershell
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl --config Release
```

The server automatically looks for:

```text
build/sitl/Release/triwhirl-standup-sitl.exe
build/sitl/Debug/triwhirl-standup-sitl.exe
```

Linux/macOS single-config builds are also supported.

## Launch

```powershell
python tools/visualization/triwhirl-sim-viewer/serve_live.py
```

Then open:

```text
http://127.0.0.1:8000/tools/visualization/triwhirl-sim-viewer/
```

If the executable is elsewhere:

```powershell
python tools/visualization/triwhirl-sim-viewer/serve_live.py `
  --exe .\build\sitl\Release\triwhirl-standup-sitl.exe
```

Press **Run 10 s gate**. Each run launches a fresh native simulation and then
replays its evidence at the selected wall-clock speed.

## What is rendered

- actual local body error from the virtual plant;
- reaction-wheel angle and speed;
- production controller phase, settling and stable flags;
- filtered body rate;
- wheel target / velocity error / PI integral;
- unclamped and applied Vq;
- rolling body-error and Vq traces;
- selected provisional plant profile;
- the native long-run simulation-gate result.

The body drawing uses the same sign as the SITL local angle. The browser does not
flip a sign or exaggerate body motion to make the animation look stable.

## 10-second simulation gate

The native executable owns the pass/fail decision. For a run of at least 10 s,
the current gate requires:

- Balance phase for the entire run;
- settling acquired;
- `stable=true` observed;
- at least one upright crossing and one applied-Vq sign reversal;
- maximum body error below 5 deg;
- final 2 s maximum body error below 0.5 deg;
- maximum reaction-wheel speed below 20 rad/s;
- applied Vq within the 4 V vector limit;
- no Vq saturation.

The WebUI only displays that native result.

## Evidence boundary

The B/C local models remain **provisional commissioning evidence**. The nominal
profile is their element-wise midpoint and is the current long-run simulation
gate profile. B and C can be selected to expose model sensitivity, but a PASS on
the nominal model is not a hardware authorization by itself.

Therefore the console permanently displays:

```text
SIMULATION ONLY
NO PHYSICAL AUTHORITY
```

The physical platform remains blocked until simulation behavior is accepted and
the remaining model/evidence questions are explicitly resolved.
