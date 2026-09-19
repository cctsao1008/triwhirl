# TriWhirl host tools

This directory contains programs that run on the development host rather than on the ESP32. Host tools may configure experiments, acquire or decode data, fit models, and generate control artifacts, but they are not part of the real-time ESP32 control loop.

## TriWhirl Toolbox

`twtool` is the canonical host-tool entry point. It presents acquisition, fitting, logging, and analysis as one command tree while legacy scripts are progressively folded into shared modules.

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

id
  actuator-uart  tethered actuator acquisition
  actuator-ble   untethered actuator acquisition
  body-free      free-body BLE acquisition
  body-local     passive local-upright acquisition
  body-active    active local-upright acquisition
  swing          autonomous reaction-wheel swing acquisition

fit
  actuator       preliminary actuator/local continuous-time fit
  body-local     passive local-upright fit
  body-active    active per-vertex A/B/C fit
```

The TWLG commands are native toolbox commands backed by shared BLE/TWLG modules. Identification/fitting commands still route to proven legacy implementations during migration.

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

Other examples:

```powershell
python tools/twtool.py id swing --probes 12 -o artifacts/auto-swing-id-01.csv
python tools/twtool.py fit body-active artifacts/body-active-B.csv -o artifacts/body-active-B-fit.json
```

Use `help` to open command-specific argument help through the unified entry point:

```powershell
python tools/twtool.py help log session
python tools/twtool.py help log inspect
python tools/twtool.py help id swing
```

The old scripts remain available during migration. New host workflows should prefer `twtool` so command naming and shared BLE/TWLG/metadata infrastructure have one stable interface.

## Design boundary

The toolbox is for commissioning, acquisition, identification, logging, analysis, and synthesis. Timing-critical control decisions belong in the ESP32 firmware. BLE is not a real-time control transport: the firmware-owned 1 kHz control/logger path records synchronized data locally and BLE is used for configuration and post-run transfer.

Do not create a separate `host/` application hierarchy unless TriWhirl later gains an actual host-side runtime application.
