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

## Current fitting and replay path

Fit and validate the vertex-agnostic local model through the toolbox:

```powershell
python tools/twtool.py plant calibrate `
  artifacts/plant-id/calibration/local.csv `
  --validation artifacts/plant-id/validation/local.csv `
  --output-dir artifacts/plant-calibration
```

The default path uses `body_active_local_fit.py` and emits `triwhirl-local-linear-model-v1`. The fitter uses each trial's recorded `theta_ref_rad`, rejects derivative windows that cross measured-Vq or phase transitions, and reports shared-fit quality plus per-trial coefficient variation. It does not classify trials by a fixed A/B/C angle.

Replay uses the same local coordinate:

```powershell
python tools/twtool.py plant replay `
  artifacts/plant-calibration/linear-model.json `
  artifacts/plant-id/validation/local.csv `
  -o artifacts/plant-calibration/replay-parity.json
```

Independent holdout replay remains required before the model-evidence gate can pass. A holdout run with no explicit empirically justified parity thresholds is `REVIEW`, not `PASS`.

Historical A/B/C-aware fitting remains available through `plant calibrate-legacy` and the old fit/converter scripts for reproducibility of earlier evidence. It is not the default route for new acquisition.

## Other acquisition tools

```text
actuator-uart   tethered actuator acquisition
actuator-ble    untethered actuator acquisition
body-free       free-body motion
body-local      passive local-upright release
body-active     signed local-upright Vq excitation
swing           firmware-owned swing identification
```

The repository intentionally does not fabricate plant coefficients. Fresh hardware calibration and an independent validation capture are still required before near-upright fuzzy tuning can be treated as physically supported.
