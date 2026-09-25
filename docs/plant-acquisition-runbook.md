# Plant Acquisition Runbook

This runbook is the hardware data-collection companion to [`plant-calibration.md`](plant-calibration.md).

The standup commissioning controller is frozen while this dataset is collected. Do not use failed standup traces as fitting data.

## 1. Preflight

Use the same firmware, motor calibration, battery setup, mechanical assembly, and IMU orientation for calibration and validation.

```powershell
python tools/twtool.py diag realtime-check 10 --baseline-seconds 5
python tools/twtool.py diag motor-direction
```

Do not proceed if realtime/sensor health or motor direction is unresolved.

## 2. Calibration acquisitions

Collect each physical upright vertex as a separate `body-active` run. The acquisition command intentionally locks one run to one vertex, so A/B/C are not mixed during handling.

Use the existing default excitation envelope first (`|Vq|=0.25 V`, 0.12 s pulse, 8 deg local limit). Six trials per vertex provide three positive and three negative trials under the current deterministic sign schedule.

```powershell
mkdir artifacts\plant-id\calibration

python tools/twtool.py id body-active --vertex A --trials 6 `
  -o artifacts\plant-id\calibration\A.csv

python tools/twtool.py id body-active --vertex B --trials 6 `
  -o artifacts\plant-id\calibration\B.csv

python tools/twtool.py id body-active --vertex C --trials 6 `
  -o artifacts\plant-id\calibration\C.csv
```

For every trial, hold the requested vertex still until firmware confirms the measured Vq is established, then release without pushing. The measured firmware `vq_v`, not host command timing, is the identification input.

If one vertex cannot be physically held/released safely or repeatably, stop and document that limitation rather than relabeling another posture.

## 3. Merge calibration runs

Each acquisition restarts trial numbering at 1. Never concatenate these CSV files manually because duplicate trial IDs would cause the fitter to mix separate runs.

```powershell
python tools/twtool.py plant merge-active `
  artifacts\plant-id\calibration\A.csv `
  artifacts\plant-id\calibration\B.csv `
  artifacts\plant-id\calibration\C.csv `
  -o artifacts\plant-id\calibration.csv
```

The merge tool renumbers trials globally and emits a `.merge.json` manifest containing input SHA-256 hashes and the old-to-new trial mapping.

## 4. Independent validation acquisitions

Power-cycle/reposition the rig and collect a second, independent set. Do not reuse calibration rows.

```powershell
mkdir artifacts\plant-id\validation

python tools/twtool.py id body-active --vertex A --trials 6 `
  -o artifacts\plant-id\validation\A.csv

python tools/twtool.py id body-active --vertex B --trials 6 `
  -o artifacts\plant-id\validation\B.csv

python tools/twtool.py id body-active --vertex C --trials 6 `
  -o artifacts\plant-id\validation\C.csv

python tools/twtool.py plant merge-active `
  artifacts\plant-id\validation\A.csv `
  artifacts\plant-id\validation\B.csv `
  artifacts\plant-id\validation\C.csv `
  -o artifacts\plant-id\validation.csv
```

Calibration and validation should use the same declared excitation/local-angle envelope unless the experiment is intentionally testing model extrapolation.

## 5. Fit + replay parity

First run without guessed parity thresholds. This produces the model and quantitative holdout errors for inspection:

```powershell
python tools/twtool.py plant calibrate `
  artifacts\plant-id\calibration.csv `
  --validation artifacts\plant-id\validation.csv `
  --vertices auto `
  --nominal mean `
  --output-dir artifacts\plant-calibration `
  --provenance-note "independent A/B/C held-release calibration and validation"
```

Review:

```text
artifacts/plant-calibration/active-fit.json
artifacts/plant-calibration/linear-model.json
artifacts/plant-calibration/replay-parity.json
artifacts/plant-calibration/replay-parity.csv
artifacts/plant-calibration/calibration-manifest.json
```

The first holdout result is a measurement, not an automatic pass. Use repeated validation/noise/repeatability runs to choose engineering thresholds; do not set the limits merely high enough to make the first result pass. Once justified, rerun with explicit limits:

```powershell
python tools/twtool.py plant calibrate `
  artifacts\plant-id\calibration.csv `
  --validation artifacts\plant-id\validation.csv `
  --vertices auto `
  --nominal mean `
  --max-theta-rollout-rmse-deg <limit> `
  --max-rate-rollout-rmse <limit> `
  --max-wheel-rollout-rmse <limit> `
  --output-dir artifacts\plant-calibration
```

Do not proceed until `calibration-manifest.json` reports all three of these as `PASS`:

```text
fit_gate.status
parity_acceptance.status
synthesis_gate.status
```

## 6. Validated H-infinity synthesis

The synthesis pipeline accepts only the validated calibration manifest. It does not refit from the raw CSV and it refuses to run if the holdout/parity/fit gate is not `PASS` or if the validated linear-model hash changed.

```powershell
python tools/twtool.py fit hinf `
  artifacts\plant-calibration\calibration-manifest.json `
  --output-dir artifacts\hinf
```

Expected outputs:

```text
artifacts/hinf/validated-polytope.json
artifacts/hinf/validated-hinf.json
artifacts/hinf/validated-balance-command.txt
artifacts/hinf/synthesis-provenance.json
```

Keep `synthesis-provenance.json` with the controller artifact. It binds the controller to the exact calibration manifest and validated linear model by SHA-256.

## 7. Standup closed-loop replay

After a plant exists, a lossless standup CSV may be used as additional diagnostic evidence:

```powershell
python tools/twtool.py plant replay `
  artifacts\plant-calibration\linear-model.json `
  artifacts\standup\standup-YYYYMMDD-HHMMSS.csv `
  --plant nominal `
  --max-angle-deg 8 `
  -o artifacts\plant-calibration\standup-parity.json `
  --csv artifacts\plant-calibration\standup-parity.csv
```

This does **not** replace independent active-ID validation. It asks whether the identified local plant reproduces the early closed-loop capture trajectory under the measured applied Vq.

## Stop conditions

Stop collection and diagnose before fitting if any of these occur:

- sensor/realtime fault during the experiment;
- incorrect motor/encoder sign;
- Vq never establishes while the body is held;
- release requires a deliberate push rather than a clean finger release;
- data repeatedly exits the local angle envelope almost immediately;
- one Vq sign is absent or materially underrepresented;
- vertex labels disagree with the measured held reference.
