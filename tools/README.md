# TriWhirl host tools

This directory contains programs that run on the development host rather than on the ESP32. Host tools may configure experiments, acquire or decode data, fit models, and generate control artifacts, but they are not part of the real-time ESP32 control loop.

## TriWhirl Toolbox

`twtool` is the canonical host-tool entry point. It presents acquisition, fitting, logging, plotting, and analysis as one command tree while legacy scripts are progressively folded into shared modules.

From the repository root:

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
  balance        deploy/run a guarded H-infinity near-upright controller
  standup        autonomous vendor-aligned swing-up -> balance (+ optional plot)

id
  actuator-uart  tethered actuator acquisition
  actuator-ble   untethered actuator acquisition
  body-free      free-body BLE acquisition
  body-local     passive local-upright acquisition
  body-active    signed per-vertex active local identification
  swing          autonomous reaction-wheel swing acquisition

plant
  merge-active   merge separate A/B/C active-ID runs with trial renumbering
  calibrate      fit + linearize + holdout replay + synthesis gate manifest
  replay         one-step and free-run replay parity against measured Vq

fit
  actuator       preliminary actuator/local continuous-time fit
  body-local     passive local-upright fit
  body-active    active per-vertex A/B/C fit
  hinf           synthesize only from a PASS calibration manifest
```

The TWLG, standup-trace, plant-calibration, and replay commands are native toolbox commands backed by shared modules. Some identification/fitting commands still route to proven legacy implementations during migration.

### Plant calibration and synthesis gate

The formal controller path is now:

```text
signed A/B/C active-ID calibration data
    -> per-vertex fit
    -> continuous local plant family
    -> independent holdout replay parity
    -> PASS calibration manifest
    -> H-infinity synthesis
```

Use separate calibration and validation acquisitions, merge A/B/C files with `plant merge-active`, then run `plant calibrate`. H-infinity synthesis no longer accepts a raw active-ID CSV as plant authority. `fit hinf` consumes only a calibration manifest whose fit, holdout parity, and synthesis gates are `PASS`, and it verifies the validated linear-model SHA-256 before invoking the solver.

See `docs/plant-acquisition-runbook.md` and `docs/plant-calibration.md` for the acquisition procedure and promotion contract.

### Firmware logger workflow

The normal logger lifecycle no longer needs a serial terminal:

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

For a simple fixed-duration recording, the same lifecycle can be collapsed into one host command. The data path is still firmware-owned; the host only starts/stops the session and downloads after capture:

```powershell
python tools/twtool.py log session 45 `
  -o artifacts/run-01.twlog `
  --csv artifacts/run-01.csv
```

`log session` reserves a small amount of extra flash capacity for host stop-command latency, then runs `prepare -> start -> wait -> stop/finalize -> download`. `Ctrl-C` still asks firmware to finalize and download the partial log before the tool exits.

`log prepare` waits for the background flash erase to reach firmware `state=ready` unless `--no-wait` is supplied. `log stop` waits until the SRAM buffer has drained, the TWLG header/CRC are finalized, and firmware reaches `state=complete`.

`log critical on` pauses flash programming without stopping 1 kHz SRAM capture; `log critical off` resumes flash writes. The future firmware-owned upright experiment supervisor will drive this automatically around critical local windows rather than relying on BLE timing.

`log inspect` reports the validated TWLG header plus useful acquisition ranges such as body angle, body rate, wheel rate, Vq, dropped records, and fault coverage. Use `--json` for a machine-readable summary.

### Standup trace and plot

`control standup` captures the dedicated binary standup trace into host RAM, then writes `.twtrace`, `.csv`, and `.json` only after the motor has stopped. The controller itself remains at 1 kHz; the trace is intentionally decimated before BLE transport so the captured stream can remain lossless. Add `--plot` to also write a four-panel PNG after the trace is safely persisted:

```powershell
python tools/twtool.py control standup --duration 10 --plot
```

The TWTR2 trace records the measured inter-sample `dt_us` from the firmware control clock rather than reconstructing time from an assumed sample period. Each frame also carries the full 160-bit git commit embedded into the flashed firmware plus a dirty-worktree flag. The JSON sidecar records the host commit separately and reports whether host and firmware revisions match.

The end-of-run `standup_trace` summary reports a strict acquisition result. `acceptance=PASS` requires START/END markers, valid CRC and frame/sample sequences, no missing/reordered samples, no ESP32 ring or BLE transport drops, no malformed/trailing bytes, no clamped/zero noninitial `dt_us`, clean firmware provenance, and an exact host/firmware git-commit match. BLE transport-drop accounting is reset logically at the start of each trace so an older failed run does not contaminate the next run.

Use `--show-plot` to save the PNG and also open it interactively. Plotting is post-run only and never participates in the BLE callback or realtime control path.

Existing standup captures can be replotted independently from either the authoritative raw trace or its decoded CSV:

```powershell
python tools/twtool.py log plot-standup artifacts/standup/standup-20260923-213000.twtrace
python tools/twtool.py log plot-standup artifacts/standup/standup-20260923-213000.csv --show
```

The plot shows upright error with the capture/release boundaries, body and filtered gyro rates, wheel rate versus target velocity, and PI/Vq behavior. Matplotlib is imported only when plotting is requested; if it is not installed, install it in the active host environment with `python -m pip install matplotlib`.

Other examples:

```powershell
python tools/twtool.py id swing --probes 12 -o artifacts/auto-swing-id-01.csv
python tools/twtool.py fit body-active artifacts/body-active-B.csv -o artifacts/body-active-B-fit.json
python tools/twtool.py plant calibrate artifacts/plant-id/calibration.csv `
  --validation artifacts/plant-id/validation.csv `
  --output-dir artifacts/plant-calibration
```

Use `help` to open command-specific argument help through the unified entry point:

```powershell
python tools/twtool.py help log session
python tools/twtool.py help control standup
python tools/twtool.py help plant merge-active
python tools/twtool.py help plant calibrate
python tools/twtool.py help plant replay
python tools/twtool.py help fit hinf
```

The old scripts remain available during migration. New host workflows should prefer `twtool` so command naming and shared BLE/TWLG/metadata infrastructure have one stable interface.

## Design boundary

The toolbox is for commissioning, acquisition, identification, logging, plotting, analysis, and synthesis. Timing-critical control decisions belong in the ESP32 firmware. BLE is not a real-time control transport: standup trace callbacks append binary notifications into host RAM without parsing/plotting/disk I/O, and visualization happens only after the trial.

Do not create a separate `host/` application hierarchy unless TriWhirl later gains an actual host-side runtime application.
