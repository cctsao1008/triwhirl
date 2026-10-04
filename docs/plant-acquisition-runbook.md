# Plant acquisition runbook

This runbook is the hardware data-collection companion to [`plant-calibration.md`](plant-calibration.md).

Near-upright fuzzy tuning is frozen while this dataset is collected. Do not change the local coordinate convention, tune around failed standup traces, or promote synthetic fixtures as physical plant evidence during this phase.

## 1. Preflight

Use the same firmware revision, motor calibration, battery setup, mechanical assembly, and IMU orientation for calibration and validation.

```powershell
python tools/twtool.py diag realtime-check 10 --baseline-seconds 5
python tools/twtool.py diag motor-direction
```

Do not proceed if realtime/sensor health or motor direction is unresolved.

## 2. Calibration acquisition

The current `body-active` path is vertex agnostic. Do **not** select A/B/C. Hold the rig at any safe, repeatable upright equilibrium; each trial records its own held reference:

```text
theta_ref   = median held attitude
theta_error = wrap(theta_rad - theta_ref)
```

Start with the existing conservative excitation envelope. The current default is signed `|Vq| = 0.25 V`; firmware limits remain authoritative.

```powershell
mkdir artifacts\plant-id\calibration

python tools/twtool.py id body-active --trials 6 `
  -o artifacts\plant-id\calibration\local-01.csv
```

For every trial:

1. hold a stable upright posture without intentionally forcing it toward a named vertex;
2. wait while the tool establishes and records a fresh `theta_ref_rad`;
3. keep holding while the signed Vq command is established and measured firmware `vq_v` is verified;
4. release cleanly without pushing;
5. let the tool capture the bounded local motion and zero-vector continuation.

The measured firmware `vq_v`, not BLE command timing, is the identification input.

If you want repeatability evidence from different physical upright orientations, reposition the body naturally and run another independent capture. The absolute held angle is provenance; it is not a controller label.

## 3. Optional merge of independent calibration runs

Each acquisition restarts trial numbering. If several runs are needed, merge them with the tool rather than concatenating CSVs manually:

```powershell
python tools/twtool.py plant merge-active `
  artifacts\plant-id\calibration\local-01.csv `
  artifacts\plant-id\calibration\local-02.csv `
  -o artifacts\plant-id\calibration\local.csv
```

`plant merge-active` renumbers trials globally, preserves every `theta_ref_rad`, records input SHA-256 hashes and old-to-new trial mappings, and does not require `vertex_id`.

If only one run is used, its CSV can be passed directly to `plant calibrate`.

## 4. Independent validation acquisition

Power-cycle and/or reposition the rig and collect a second dataset under the same declared excitation/local-angle envelope. Do not reuse calibration rows.

```powershell
mkdir artifacts\plant-id\validation

python tools/twtool.py id body-active --trials 6 `
  -o artifacts\plant-id\validation\local-01.csv
```

Additional validation runs can be merged in the same way:

```powershell
python tools/twtool.py plant merge-active `
  artifacts\plant-id\validation\local-01.csv `
  artifacts\plant-id\validation\local-02.csv `
  -o artifacts\plant-id\validation\local.csv
```

Calibration and validation should use the same operating envelope unless the experiment is explicitly testing extrapolation. Changing physical upright orientation is allowed and useful for repeatability evidence because the local coordinate is re-established per trial.

## 5. Fit and holdout replay

First run without guessed parity thresholds. This produces a model and quantitative holdout errors for inspection:

```powershell
python tools/twtool.py plant calibrate `
  artifacts\plant-id\calibration\local.csv `
  --validation artifacts\plant-id\validation\local.csv `
  --output-dir artifacts\plant-calibration `
  --provenance-note "independent vertex-agnostic local calibration and validation"
```

Review:

```text
artifacts/plant-calibration/active-local-fit.json
artifacts/plant-calibration/linear-model.json
artifacts/plant-calibration/replay-parity.json
artifacts/plant-calibration/replay-parity.csv
artifacts/plant-calibration/calibration-manifest.json
```

The first holdout result is a measurement, not an automatic pass. Review sample coverage, conditioning, fitted coefficients, per-trial variation, residuals, and replay parity. Use repeated validation/noise/repeatability evidence to choose engineering thresholds; do not set limits merely high enough to make the first result pass.

Once empirical limits are justified, rerun with explicit limits:

```powershell
python tools/twtool.py plant calibrate `
  artifacts\plant-id\calibration\local.csv `
  --validation artifacts\plant-id\validation\local.csv `
  --max-theta-rollout-rmse-deg <limit> `
  --max-rate-rollout-rmse <limit> `
  --max-wheel-rollout-rmse <limit> `
  --output-dir artifacts\plant-calibration
```

Interpret the current v2 manifest gates as:

```text
fit_gate.status          model structure/excitation check
parity_acceptance.status explicit holdout thresholds, if supplied
promotion_gate.status    local model-evidence promotion state
```

A holdout run with no quantitative thresholds is `REVIEW`. Missing holdout data or a failed fit/parity gate is `BLOCKED`. A `PASS` is still only a model-evidence result; it does not authorize standup hardware control.

## 6. Handoff to full-fuzzy development

The target attitude-control path is:

```text
[theta_error, theta_rate, wheel_velocity]
                |
                v
       full-fuzzy attitude control
                |
                v
       target_velocity [rad/s]
                |
                v
       SimpleFOC velocity control
```

The local plant is identified in measured `Vq` because that is the current observable actuator coordinate. Before tuning the fuzzy `target_velocity` controller against this plant, obtain motor velocity-servo evidence that supports the mapping from target velocity to wheel response. Do not inject `target_velocity` into a Vq plant as though the units were interchangeable.

The historical H-infinity pipeline is not the current target-controller handoff. Its v1 calibration manifest path remains in the repository for reproducibility only.

## 7. Closed-loop trace replay

After a physical local plant exists, a lossless standup/balance CSV may be used as additional diagnostic evidence:

```powershell
python tools/twtool.py plant replay `
  artifacts\plant-calibration\linear-model.json `
  artifacts\standup\standup-YYYYMMDD-HHMMSS.csv `
  --max-angle-deg 8 `
  -o artifacts\plant-calibration\standup-parity.json `
  --csv artifacts\plant-calibration\standup-parity.csv
```

This does **not** replace independent active-ID validation. It asks whether the identified local plant reproduces the early closed-loop local trajectory under measured applied Vq.

## Stop conditions

Stop collection and diagnose before fitting if any of these occur:

- sensor/realtime fault during the experiment;
- incorrect motor/encoder sign;
- measured `vq_v` does not establish while the body is held;
- release requires a deliberate push rather than a clean release;
- data repeatedly exits the local angle envelope almost immediately;
- one Vq sign is absent or materially underrepresented;
- held posture is not stable enough for a meaningful per-trial `theta_ref`;
- calibration and validation hardware/firmware conditions are no longer comparable.
