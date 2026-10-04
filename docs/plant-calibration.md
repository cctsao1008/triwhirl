# TriWhirl local plant calibration and replay parity

This document defines the evidence gate between near-upright hardware identification and tuning of the target full-fuzzy attitude controller.

The current phase is **plant calibration**, not balance tuning. The repository must first establish a repeatable local physical model from measured motion and measured firmware `Vq`, validate that model on independent data, and only then use the resulting evidence to constrain fuzzy normalization/rule tuning and motor-layer experiments.

## Current state and input contract

The controller and identification model share one vertex-agnostic local upright coordinate:

```text
x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
u = measured firmware Vq_v

theta_ref   = median held attitude for this trial
theta_error = wrap(theta_rad - theta_ref)
```

Every active-ID trial establishes a fresh `theta_ref_rad` from the actual held posture. The absolute angle is retained as provenance and repeatability evidence, but it is **not** converted into an A/B/C controller identity and is not compared with a fixed nominal vertex such as 68 degrees.

Different physical upright orientations may therefore be used across trials. Any orientation-dependent variation that appears in the fitted coefficients belongs in repeatability/model-uncertainty analysis.

Measured firmware `Vq_v` is the authoritative identification input. BLE command timing is not used as the plant input timestamp.

## Acquisition

Use the current vertex-agnostic active acquisition path:

```powershell
python tools/twtool.py id body-active --trials 6 `
  -o artifacts/plant-id/calibration/local.csv
```

The acquisition calibrates/initializes the IMU once, then each trial:

1. waits for a stable held upright posture;
2. records a fresh `theta_ref_rad`;
3. establishes the requested signed `Vq` while the body is still held;
4. verifies the measured firmware `vq_v` from fresh telemetry;
5. asks the operator to release without pushing;
6. records the bounded local departure and zero-vector continuation.

The default signed excitation is `+0.25 V / -0.25 V`. Actual admissible excitation remains a hardware-safety decision; acquisition must stay within firmware limits.

Collect calibration and validation datasets independently:

```text
calibration/local.csv  -> parameter estimation
validation/local.csv   -> replay parity only
```

Do not fit and validate on the same samples and call the model validated.

## Default calibration pipeline

The current one-command path is:

```powershell
python tools/twtool.py plant calibrate `
  artifacts/plant-id/calibration/local.csv `
  --validation artifacts/plant-id/validation/local.csv `
  --output-dir artifacts/plant-calibration
```

The default route performs:

```text
body-active local CSV
        |
        v
vertex-agnostic held-input fit
        |
        v
triwhirl-local-linear-model-v1
        |
        v
independent holdout replay
        |
        v
calibration-manifest-v2
```

Generated artifacts are:

```text
active-local-fit.json
linear-model.json
replay-parity.json
replay-parity.csv
calibration-manifest.json
```

The manifest records SHA-256 hashes of the calibration/validation inputs and generated evidence artifacts. These hashes provide engineering provenance and accidental-drift detection; they are not cryptographic attestation of physical validity.

If `--validation` is omitted, an in-sample replay is still emitted for diagnosis, but the promotion gate is `BLOCKED`.

## Local fit contract

`tools/parameter_id/body_active_local_fit.py` uses the recorded per-trial `theta_ref_rad` directly. For every derivative window it rejects:

- invalid/faulted telemetry;
- non-monotonic timestamps;
- windows crossing an acquisition phase boundary;
- windows crossing a measured `Vq` transition;
- samples outside the configured local-angle envelope.

The shared continuous-time regression is:

```text
theta_ddot = a_theta*theta_error
           + a_rate*theta_rate
           + a_wheel*wheel_rate
           + b_vq*Vq
           + body_bias

wheel_accel = c_theta*theta_error
            + c_rate*theta_rate
            + c_wheel*wheel_rate
            + d_vq*Vq
            + wheel_bias
```

A fit is not promoted merely because least squares returned coefficients. The fitter reports rank, normalized conditioning, positive/negative `Vq` coverage, positive/negative local-angle coverage, Vq-coefficient statistical separation, per-trial fit diagnostics, and observed coefficient variation across independently fitted full-rank trials.

The affine biases are retained for diagnostics and replay. They are not interpreted as equilibrium control commands.

## Linear-model artifact

`model/linearization/from_local_fit.py` converts the accepted shared fit to:

```text
xdot = A*x + Bv*Vq + bias
```

with explicit artifact format:

```text
triwhirl-local-linear-model-v1
```

The artifact records:

- state order and measured-input contract;
- `active_error_mode = wrapped_theta_ref`;
- nominal `A`, `Bv`, and affine-bias diagnostics;
- open-loop eigenvalues;
- controllability rank/condition;
- calibration trial `theta_ref` provenance;
- observed per-trial coefficient variation.

It contains no required A/B/C plant selection.

## Replay parity

The default replay command accepts both the current local artifact and historical vertex-aware artifacts:

```powershell
python tools/twtool.py plant replay `
  artifacts/plant-calibration/linear-model.json `
  artifacts/plant-id/validation/local.csv `
  -o artifacts/plant-calibration/replay-parity.json `
  --csv artifacts/plant-calibration/replay-parity.csv
```

For `triwhirl-local-linear-model-v1`, active-ID replay reconstructs the measured state with the same acquisition contract:

```text
theta_error = wrap(theta_rad - theta_ref_rad)
```

It never selects a plant from absolute angle. Historical `triwhirl-vertex-normalized-linear-model-v1` artifacts continue to use the old A/B/C-aware replay path solely so earlier evidence remains reproducible.

For each valid replay interval the harness computes:

1. **one-step prediction**: initialize from the previous measured state and integrate one measured sample interval;
2. **free-run rollout**: initialize once at the trial entry and recursively propagate using measured `Vq`.

The numerical integrator is RK4 with measured sample intervals and zero-order-held measured `Vq`.

## Fit and parity gates

For the current local artifact, the calibration fit gate requires:

- the pooled active-ID fit has `candidate` status;
- the declared three-state local model is controllable at rank 3;
- the fitted upright model retains a finite open-loop unstable mode, as expected for the near-upright operating point.

Replay parity alone is not enough. Independent holdout data are required before promotion. Likewise, the repository does **not** invent universal RMSE limits before hardware repeatability and sensor noise establish defensible thresholds.

Therefore:

- no validation dataset -> `BLOCKED`;
- holdout exists but thresholds are unset -> `REVIEW`;
- supplied thresholds fail -> `BLOCKED`;
- candidate fit + independent holdout + explicit thresholds pass -> `PASS` for the **model-evidence gate only**.

A `PASS` here is not proof of hardware balance and does not authorize motor energization or standup trials by itself.

## Relationship to the full-fuzzy architecture

The target system controller remains:

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

`Vq` remains appropriate for plant identification because it is the measurable actuator coordinate of the current commissioning runtime. It is **not** the production fuzzy-controller output.

Once fresh calibration and holdout evidence are coherent, the local model can be transformed through the measured/validated motor velocity-servo behavior and used to constrain fuzzy scaling and robustness tests. That step must use actual motor-loop evidence rather than treating the wheel as an ideal acceleration source.

## Historical compatibility

The following remain in the repository for reproducibility of earlier experiments:

- `tools/parameter_id/body_active_fit.py` — historical A/B/C-aware fitter;
- `model/linearization/from_active_fit.py` — historical vertex-normalized converter;
- `python tools/twtool.py plant calibrate-legacy ...` — historical calibration route;
- `triwhirl-vertex-normalized-linear-model-v1` replay support;
- the legacy H-infinity synthesis pipeline and its v1 calibration manifest contract.

These historical paths are not the default for new hardware acquisition and do not define the target full-fuzzy controller architecture.

## What still requires hardware

The repository intentionally does not fabricate physical plant parameters. Issue #12 remains open until fresh near-upright calibration and independent holdout captures establish a coherent model family and empirical parity limits. Until then, historical B/C fixtures and synthetic CI data remain software/modeling evidence only, and near-upright fuzzy tuning remains blocked.
