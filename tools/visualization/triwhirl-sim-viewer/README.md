# TriWhirl Simulation Console

Local WebUI for the native standup SITL.

The default interactive scenario now starts from a resting face and shows the
whole controller sequence instead of spawning already upright:

```text
rest -> swing-up -> capture -> settling -> stable balance
```

The console also keeps the previous upright-only local-balance scenario for
focused disturbance testing.

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
   +-> selected plant @ RK4 100 us
   +-> body/wheel disturbance impulses
```

The browser contains no controller equations or plant integrator. It is a
display/control surface only.

### Full standup plant

`Full standup from rest` uses a hybrid commissioning model:

- far from upright: explicit 120-degree periodic swing-up surrogate;
- within +/-2 deg: selected provisional identified local B/C plant;
- 2..5 deg: smooth transition between them.

The far-field surrogate provides gravity-like rocking, damping and actuator
coupling needed to exercise `SwingHigh -> SwingLow -> Balance`. It is **not** a
validated model of the actual Reuleaux rolling/contact geometry.

### Upright balance plant

`Upright balance only` retains the original identified local B/C model and starts
near upright for focused balance/disturbance work.

## Build

From the repository root on Windows:

```powershell
cmake -S tools/sitl -B build/sitl
cmake --build build/sitl --config Release
```

This builds:

```text
triwhirl-standup-sitl                 deterministic local balance gate
triwhirl-standup-sitl-full            deterministic full standup gate
triwhirl-standup-sitl-disturbance     deterministic disturbance gate
triwhirl-standup-sitl-live            continuous interactive native session
```

## Launch

```powershell
python tools/visualization/triwhirl-sim-viewer/serve_live.py
```

Open:

```text
http://127.0.0.1:8000/tools/visualization/triwhirl-sim-viewer/
```

Select a scenario and press **Run live**. The default is **Full standup from
rest**. There is no interactive duration limit; the native process runs until
**Stop**.

The rolling plot displays only the most recent 10 seconds. That is a display
window, not a simulation lifetime.

## What full standup should show

A nominal successful run visibly proceeds through:

```text
resting face near -59 deg periodic error
    -> SwingHigh pumping
    -> reversal and SwingLow/SwingHigh pumping
    -> capture near upright
    -> Balance
    -> settling
    -> Stable
```

The right panel shows both the wrapped 120-degree periodic upright error and the
unwrapped body angle. Keeping both prevents the renderer from visually snapping
at a periodic boundary.

The drawing also shifts vertically with body orientation so the rendered
Reuleaux boundary stays on the ground line while it rocks. This display contact
support is geometric visualization only; it is not a contact solver.

## Interactive disturbances

While a session is running:

- **Body push Delta theta-dot** adds a signed body angular-rate impulse to the
  current native state.
- **Wheel kick Delta omega** adds a signed reaction-wheel-rate impulse to the
  current native state.

The already-running C++ process applies the disturbance immediately. There is no
browser-only fake motion and no trajectory rewind/recompute.

## Deterministic CI gates

Interactive runs are unlimited, but CI retains finite deterministic regression
gates:

- local 10-second balance gate;
- deterministic disturbance-recovery gate;
- full 12-second rest-to-upright gate;
- live bridge smoke test that observes swing, Balance and Stable, injects a
  disturbance, then stops the native process.

These gates are repeatable regression evidence, not a WebUI runtime limit.

## Evidence boundary

The B/C local models remain provisional commissioning evidence, and the full
standup far-field dynamics are an explicit periodic surrogate. Therefore the
console permanently remains simulation-only and provides **no physical
authority** by itself. Its purpose is to catch controller/state-machine and
robustness failures before spending another hardware trial.
