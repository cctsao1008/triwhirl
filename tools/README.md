# TriWhirl host tools

This directory contains programs that run on the development host rather than on the ESP32. Host tools may configure experiments, acquire or decode data, fit models, and generate analysis artifacts, but they are not part of the realtime ESP32 control loop.

## TriWhirl Toolbox

`twtool` is the canonical host-tool entry point. From the repository root:

```powershell
python tools/twtool.py --help
python tools/twtool.py --list
```

The package entry point is equivalent:

```powershell
python -m tools.triwhirl_tool --help
```

Current command tree:

```text
log
  status         firmware TWLG logger state over BLE
  prepare        pre-erase/prepare flash before a realtime run
  start          start synchronized 1 kHz TWLG capture
  critical       pause/resume flash programming while SRAM capture continues
  stop           stop capture and wait for flash/header finalization
  session        prepare + record + finalize + download (+ optional decode)
  capture-uart   live UART telemetry -> CSV
  download       completed firmware TWLG -> .twlog over BLE
  decode         validate/decode .twlog -> CSV
  inspect        validate + summarize a .twlog without converting it
  plot-standup   plot a standup .twtrace or decoded standup CSV

control
  balance        legacy guarded H-infinity commissioning path
  standup        legacy commissioning swing-up/balance path

id
  actuator-uart  tethered actuator acquisition
  actuator-ble   untethered actuator acquisition
  body-free      free-body BLE acquisition
  body-local     passive local-upright acquisition
  body-active    signed vertex-agnostic local identification; fresh theta_ref each trial
  swing          firmware-owned reaction-wheel swing acquisition

plant
  merge-active       merge independent local active-ID runs with trial renumbering
  calibrate          current vertex-agnostic fit + linear model + holdout replay
  calibrate-legacy   historical A/B/C-aware calibration route
  replay             replay current local or historical vertex-aware model artifacts

fit
  actuator           preliminary actuator/local continuous-time fit
  body-local         passive local-upright fit
  body-active-local  current shared vertex-agnostic active local fit
  body-active        historical A/B/C-aware active fit
  hinf               historical H-infinity synthesis from a compatible v1 manifest
```

The target system architecture is full-fuzzy attitude control emitting bounded `target_velocity [rad/s]` into SimpleFOC velocity control. The legacy `control balance`, `control standup`, and H-infinity tooling remain useful for historical evidence and commissioning/reproducibility, but they do not define the target attitude-control architecture.

## Current local plant calibration gate

The active modeling blocker is fresh near-upright physical evidence. New active-ID data use one local coordinate independent of A/B/C labels:

```text
x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
u = measured firmware Vq_v

theta_ref   = median held attitude for each trial
theta_error = wrap(theta_rad - theta_ref)
```

Acquire independent calibration and validation datasets:

```powershell
python tools/twtool.py id body-active --trials 6 `
  -o artifacts/plant-id/calibration/local.csv

python tools/twtool.py id body-active --trials 6 `
  -o artifacts/plant-id/validation/local.csv
```

If several independent runs need to be combined, `plant merge-active` globally renumbers trials and preserves each recorded `theta_ref_rad`; it does not require a `vertex_id`:

```powershell
python tools/twtool.py plant merge-active `
  artifacts/run-01.csv artifacts/run-02.csv `
  -o artifacts/plant-id/calibration/local.csv
```

Then fit and evaluate holdout replay:

```powershell
python tools/twtool.py plant calibrate `
  artifacts/plant-id/calibration/local.csv `
  --validation artifacts/plant-id/validation/local.csv `
  --output-dir artifacts/plant-calibration
```

The current calibration route emits:

```text
active-local-fit.json
linear-model.json              # triwhirl-local-linear-model-v1
replay-parity.json
replay-parity.csv
calibration-manifest.json      # triwhirl-plant-calibration-v2
```

The v2 manifest is **model-evidence provenance only**. It is deliberately not accepted by the legacy H-infinity synthesis gate. A separate validation dataset is mandatory before model promotion; if empirical replay thresholds have not yet been established, holdout parity is `REVIEW`, not `PASS`.

Synthetic CI fixtures prove only coordinate/tool behavior. They do not provide hardware plant coefficients or authorize fuzzy tuning.

Historical A/B/C evidence remains reproducible through `plant calibrate-legacy` and the old `triwhirl-vertex-normalized-linear-model-v1` replay path.

See `docs/plant-acquisition-runbook.md`, `docs/plant-calibration.md`, and `tools/parameter_id/README.md` for the acquisition and promotion contracts.

## Firmware logger workflow

The normal logger lifecycle does not require a serial terminal:

```powershell
python tools/twtool.py log status
python tools/twtool.py log prepare 45
python tools/twtool.py log start
# realtime experiment runs on ESP32
python tools/twtool.py log stop
python tools/twtool.py log download -o artifacts/run-01.twlog
python tools/twtool.py log inspect artifacts/run-01.twlog
python tools/twtool.py log decode artifacts/run-01.twlog -o artifacts/run-01.csv
```

For a fixed-duration recording:

```powershell
python tools/twtool.py log session 45 `
  -o artifacts/run-01.twlog `
  --csv artifacts/run-01.csv
```

`log session` drives `prepare -> start -> wait -> stop/finalize -> download`. `Ctrl-C` still asks firmware to finalize and download the partial log before the tool exits.

`log prepare` waits for background flash erase to reach firmware `state=ready` unless `--no-wait` is supplied. `log stop` waits until the SRAM buffer has drained, the TWLG header/CRC are finalized, and firmware reaches `state=complete`.

`log critical on` pauses flash programming without stopping 1 kHz SRAM capture; `log critical off` resumes flash writes.

`log inspect` reports the validated TWLG header plus acquisition ranges such as body angle, body rate, wheel rate, Vq, dropped records, and fault coverage. Use `--json` for a machine-readable summary.

## Standup trace and plot

The legacy commissioning `control standup` path captures a dedicated binary standup trace into host RAM, then writes `.twtrace`, `.csv`, and `.json` only after the motor has stopped. Trace transport is decimated so BLE handling does not become part of the realtime control loop.

```powershell
python tools/twtool.py control standup --duration 10 --plot
```

The TWTR2 trace records measured `dt_us`, firmware git provenance, and acquisition integrity. Plotting occurs only after the trace is persisted.

Existing captures can be replotted independently:

```powershell
python tools/twtool.py log plot-standup artifacts/standup/run.twtrace
python tools/twtool.py log plot-standup artifacts/standup/run.csv --show
```

Failed closed-loop standup traces are validation/diagnostic evidence after a plant model exists; do not repeatedly refit the plant from failed standup runs.

## Useful command help

```powershell
python tools/twtool.py help id body-active
python tools/twtool.py help plant merge-active
python tools/twtool.py help plant calibrate
python tools/twtool.py help plant calibrate-legacy
python tools/twtool.py help plant replay
python tools/twtool.py help fit body-active-local
```

Legacy scripts remain available during migration. New host workflows should prefer `twtool` so command naming and shared metadata infrastructure have one stable interface.

## Design boundary

The toolbox is for commissioning, acquisition, identification, logging, plotting, and analysis. Timing-critical control decisions belong in ESP32 firmware. BLE is not a realtime control transport.

Do not create a separate `host/` application hierarchy unless TriWhirl later gains an actual host-side runtime application.
