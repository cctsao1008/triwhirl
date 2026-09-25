# TriWhirl Plant Calibration and Replay Parity

This document defines the plant-modeling gate between hardware commissioning and H-infinity synthesis.

The objective is not to tune the standup controller until the hardware appears to balance. The objective is to build a local physical model that can reproduce measured near-upright motion under the same measured actuator input, validate that model on data not used for fitting, and only then promote it into robust-control synthesis.

## State and input contract

The local plant uses the shared 120-degree-periodic upright coordinate:

```text
x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
u = measured Vq_v
```

The three physical upright vertices A/B/C are identification locations. They are not three controller coordinate systems. Each vertex is fit separately because the real mass distribution can break geometric three-fold symmetry; the resulting plants form the plant family used for nominal/uncertainty construction.

Measured firmware `Vq` is the authoritative plant input. Host BLE command timing is never treated as the realtime input timestamp.

## Workflow

```text
hardware checks
    |
    +-- motor direction / FOC / encoder / IMU / 1 kHz timing
    |
plant acquisition
    |
    +-- actuator response
    +-- passive body/free response
    +-- signed active local excitation at A/B/C
    |
per-vertex fit
    |
    +-- theta_ddot model
    +-- wheel_accel model
    |
continuous local plants
    |
replay parity
    |
    +-- one-step prediction error
    +-- free-run rollout error
    |
independent holdout gate
    |
validated calibration manifest
    |
nominal + uncertainty model
    |
H-infinity synthesis
    |
guarded hardware deployment
```

## Acquisition order

Use the existing tools in this order:

```powershell
python tools/twtool.py diag realtime-check 10 --baseline-seconds 5
python tools/twtool.py diag motor-direction

python tools/twtool.py id actuator-ble ...
python tools/twtool.py id body-free ...
python tools/twtool.py id body-local ...
python tools/twtool.py id body-active ...
```

The active dataset must contain both positive and negative measured Vq excitation and useful local-angle coverage. A/B/C data must remain labeled and must not be pooled before per-vertex fitting.

For formal model promotion, collect two independent active datasets with the same experiment envelope:

```text
calibration.csv   -> parameter estimation
validation.csv    -> replay parity only
```

Do not fit and validate on the same samples and then call the result validated.

## One-command calibration

The toolbox calibration command runs the existing active fitter, converts accepted vertex fits into the shared continuous state-space model, evaluates the plant fit gate, and performs replay parity:

```powershell
python tools/twtool.py plant calibrate artifacts/calibration.csv `
  --validation artifacts/validation.csv `
  --vertices auto `
  --nominal mean `
  --output-dir artifacts/plant-calibration
```

Generated artifacts:

```text
active-fit.json
linear-model.json
replay-parity.json
replay-parity.csv
calibration-manifest.json
```

The manifest records SHA-256 hashes of the calibration and validation inputs and of the generated fit/model/parity artifacts. H-infinity synthesis consumes this manifest and verifies the validated linear-model hash before synthesis. The manifest is provenance metadata, not a cryptographic signature; its purpose is reproducibility and accidental-drift detection inside the engineering workflow.

If `--validation` is omitted, the command still emits an in-sample diagnostic replay, but the synthesis gate is deliberately marked `BLOCKED`.

### Fit gate

Replay parity alone is not enough. The selected plants must also satisfy the identification/model checks required for robust synthesis. The calibration command therefore blocks promotion unless every selected plant:

- came from a `candidate` active-ID fit rather than `diagnostic_only` data;
- is controllable at rank 3 in the declared three-state model;
- retains an open-loop unstable mode at the upright equilibrium;
- contributes to a nominal plant that is also rank-3 controllable and open-loop unstable.

These checks catch the failure mode where a numerically generated linear model replays acceptably for one dataset but no longer represents the unstable controllable plant that the controller is supposed to stabilize.

## Replay parity

Replay can also be run independently:

```powershell
python tools/twtool.py plant replay `
  artifacts/plant-calibration/linear-model.json `
  artifacts/validation.csv `
  -o artifacts/plant-calibration/replay-parity.json `
  --csv artifacts/plant-calibration/replay-parity.csv
```

The harness supports:

- fit-ready active-identification CSV, replayed per trial and per physical vertex;
- standup CSV, replayed only over contiguous local `balance` segments for diagnostic comparison.

For every replay interval the harness computes both:

1. **one-step prediction**: start from the measured state at the previous sample, integrate the plant for one measured `dt`, and compare with the next measured state;
2. **free-run rollout**: initialize once at the segment entry and recursively propagate the model using measured Vq.

One-step error diagnoses local equation fidelity. Rollout error exposes accumulated model mismatch and is the more important parity measure for an unstable plant.

The numerical integrator is RK4 using the measured sample interval and zero-order-held measured Vq.

## Parity gates

Do not invent universal RMSE thresholds before the sensor noise floor and repeatability are measured. The replay command therefore reports metrics without declaring PASS by default.

Once empirical limits are established, enforce them explicitly:

```powershell
python tools/twtool.py plant calibrate artifacts/calibration.csv `
  --validation artifacts/validation.csv `
  --max-theta-rollout-rmse-deg <limit> `
  --max-rate-rollout-rmse <limit> `
  --max-wheel-rollout-rmse <limit> `
  --output-dir artifacts/plant-calibration
```

A plant may proceed to H-infinity synthesis only when all of the following are true:

- the fit itself satisfies the sample/rank/excitation/significance gates;
- the selected linear plants pass the fit gate above;
- replay uses an independent holdout dataset;
- the validation experiment stays inside the declared local-angle validity envelope;
- explicit parity thresholds are supplied and pass;
- A/B/C variation is carried into the uncertainty model rather than silently averaged away.

A holdout replay with no thresholds is `REVIEW`, not `PASS`.

## Standup traces are validation evidence, not primary fitting data

The lossless standup recorder is useful for closed-loop diagnostic replay after a plant exists. It should not replace dedicated local identification data because standup trajectories are controller-correlated and frequently include target/Vq saturation and hybrid phase transitions.

Use standup traces to answer questions such as:

```text
Does the identified plant reproduce the first 50-200 ms after capture?
Does simulated body acceleration have the correct sign and magnitude?
Does the model predict wheel momentum accumulation seen on hardware?
Where does parity break as the trajectory leaves the local envelope?
```

Do not refit the plant from every failed standup trial. If parity fails, return to acquisition/model structure first.

## H-infinity handoff

H-infinity synthesis no longer accepts a raw active-ID CSV as its authority. It consumes only a calibration manifest whose synthesis gate is `PASS`:

```powershell
python tools/twtool.py fit hinf `
  artifacts/plant-calibration/calibration-manifest.json `
  --output-dir artifacts/hinf
```

Before synthesis, the pipeline verifies:

- `validation_mode == external_holdout`;
- `fit_gate.status == PASS`;
- `parity_acceptance.status == PASS`;
- `synthesis_gate.status == PASS`;
- the validated `linear-model.json` SHA-256 still matches the manifest.

It then builds the empirical A/B/C polytopic uncertainty model from that exact validated linear model, runs H-infinity synthesis, emits the balance deployment command, and writes `synthesis-provenance.json` containing the manifest/model/output hashes.

This is deliberately a hard gate: if the calibration is in-sample, thresholds are unset/failed, a selected fit is diagnostic-only, or the validated model has changed, synthesis stops before the solver is invoked.
