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

`log download`, `log decode`, and `log inspect` are now native toolbox commands backed by shared BLE/TWLG modules rather than subprocess wrappers. The identification/fitting commands still route to their proven legacy implementations during migration.

Examples:

```powershell
python tools/twtool.py id swing --probes 12 -o artifacts/auto-swing-id-01.csv
python tools/twtool.py fit body-active artifacts/body-active-B.csv -o artifacts/body-active-B-fit.json
python tools/twtool.py log download -o artifacts/run-01.twlog
python tools/twtool.py log inspect artifacts/run-01.twlog
python tools/twtool.py log decode artifacts/run-01.twlog -o artifacts/run-01.csv
```

`log inspect` reports the validated TWLG header plus useful acquisition ranges such as body angle, body rate, wheel rate, Vq, dropped records, and fault coverage. Use `--json` when a machine-readable summary is useful.

Use `help` to open command-specific argument help through the unified entry point:

```powershell
python tools/twtool.py help id swing
python tools/twtool.py help log decode
python tools/twtool.py help log inspect
```

The old scripts remain available during migration. New host workflows should prefer `twtool` so command naming and shared BLE/TWLG/metadata infrastructure have one stable interface.

## Design boundary

The toolbox is for commissioning, acquisition, identification, logging, analysis, and synthesis. Timing-critical control decisions belong in the ESP32 firmware. In particular, BLE is not a real-time control transport: the firmware-owned 1 kHz control/logger path records synchronized data locally and BLE is used after the run for bulk transfer.

Do not create a separate `host/` application hierarchy unless TriWhirl later gains an actual host-side runtime application.
