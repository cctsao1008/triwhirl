# TriWhirl parameter identification tools

These host-side tools acquire and fit actuator and body dynamics for the TriWhirl reaction-wheel platform.

## Current identification contract

The controller and plant state use a **local upright coordinate**, not an A/B/C vertex identity:

```text
x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
u = measured firmware Vq_v
```

For active near-upright identification, each trial defines its own equilibrium from the stable held posture:

```text
theta_ref = median held attitude
theta_error = wrap(theta - theta_ref)
```

Any physical upright orientation may be used. The absolute `theta_ref_rad` is retained as provenance, but the operator does not need to select or label A/B/C. Manufacturing asymmetry between physical orientations belongs in repeatability/model-uncertainty analysis rather than in the controller coordinate system.

## Active local upright acquisition

Use the toolbox entry point:

```powershell
python tools/twtool.py id body-active --trials 6 `
  -o artifacts/plant-id/calibration/local.csv
```

The acquisition performs one gyro calibration and one gravity-based attitude initialization before telemetry starts. It does **not** reset attitude once per trial. Instead, every trial obtains a fresh `theta_ref_rad` from a stable hold, establishes the requested signed Vq while the body is still held, verifies measured firmware `vq_v`, and then asks for release.

The default signed excitation is `+0.25 V / -0.25 V`. The measured telemetry `vq_v`, not BLE command timing, is the identification input.

Collect calibration and validation runs independently. Failed standup traces are useful closed-loop evidence but are not primary plant-fitting data.

## Other acquisition tools

```text
actuator-uart   tethered actuator acquisition
actuator-ble    untethered actuator acquisition
body-free       free-body motion
body-local      passive local-upright release
body-active     signed local-upright Vq excitation
swing           firmware-owned swing identification
```

## Fitting and replay

The plant-calibration workflow is being migrated to the same local-upright contract. Existing historical A/B/C-aware fit artifacts remain reproducible from git history, but new hardware acquisition should use the vertex-agnostic `body-active` path above.

Do not promote a plant to H-infinity synthesis solely because a least-squares fit is full rank. Use independent holdout replay and explicit parity thresholds before synthesis promotion.
