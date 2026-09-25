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

The toolbox calibration command runs the existing active fitter, converts accepted vertex fits into the shared continuous state-space model, and performs replay parity:

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

The manifest records SHA-256 hashes of the calibration and validation inputs so a synthesis artifact can be traced back to exact datasets.

If `--validation` is omitted, the command still emits an in-sample diagnostic replay, but the synthesis gate is deliberately marked `BLOCKED`.

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
python tools/twtool.py plant replay model.json validation.csv `
  -o parity.json `
  --max-theta-rollout-rmse-deg <limit> `
  --max-rate-rollout-rmse <limit> `
  --max-wheel-rollout-rmse <limit>
```

A plant may proceed to H-infinity synthesis only when all of the following are true:

- the fit itself satisfies the existing sample/rank/excitation/significance gates;
- replay uses an independent holdout dataset;
- the validation experiment stays inside the declared local-angle validity envelope;
- the supplied parity thresholds pass;
- the model preserves the observed actuator sign and unstable/stable mode structure;
- A/B/C variation is carried into the uncertainty model rather than silently averaged away.

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

Only after holdout parity passes should the existing synthesis path be used:

```powershell
python tools/twtool.py fit hinf artifacts/calibration.csv ...
```

The robust-control model should then be derived from the validated per-vertex plant family, not from manually tuned standup gains.
