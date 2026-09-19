# Local parameter identification

TriWhirl keeps acquisition and model fitting separate. Firmware owns motor/safety limits and is now the timing authority for control/identification. BLE remains useful for setup, diagnostics, and post-run transfer, but host BLE arrival time is not used as a realtime measurement clock.

## Python dependencies

Install the host-side dependencies into the active Python environment once:

```powershell
python -m pip install -r tools/parameter_id/requirements.txt
```

This installs NumPy for fitting, pySerial for UART acquisition, and Bleak for untethered BLE communication.

## Three upright vertices

The Reuleaux body has three legitimate upright vertex equilibria separated by 120 body degrees. Identification must therefore treat contact mode as part of the plant state rather than assuming there is only one valid upright orientation.

The current IMU-frame naming anchor is approximately:

```text
A ~=  +68 deg
B ~=  -52 deg
C ~= -172 deg   (equivalent to +188 deg)
```

These are classification centers, not a claim that the real mass distribution is perfectly symmetric. The real PCB, battery, motor, and wheel may make the three local plants differ. Active plant fits are therefore kept separate as A/B/C and can later form a nominal-plus-uncertainty or polytopic robust-control model.

`vertex_geometry.py` centralizes the 120-degree geometry and circular-angle classification.

## Realtime TWLG path

Host-side BLE control exposed 60--120 ms command delay around the upright crossing, while the measured passive upright e-folding time is only about 56 ms. The realtime identification path therefore no longer sends timing-critical motor decisions or samples through the PC.

The firmware records a fixed 32-byte state/actuation snapshot from the 1 kHz control cycle into SRAM. A low-priority writer transfers page-sized batches to a dedicated raw flash partition outside critical local windows. Near a vertex, flash programming can be paused completely while records continue accumulating in SRAM. After the run the frozen binary log is downloaded over BLE.

The detailed binary contract is in `docs/logging/twlog-v1.md`.

Firmware shell workflow:

```text
log status
log prepare [seconds]
log start
log critical on|off
log stop
log dump
```

`log prepare` pre-erases the bounded flash region before recording. `log start` begins 1 kHz SRAM capture. `log critical on` marks a realtime-sensitive window and pauses flash programming; this command is infrastructure for commissioning and will be driven internally by the native autonomous swing state machine rather than by a PC during the final experiment. `log stop` drains SRAM and writes the versioned header/CRC.

Post-run download and decode:

```powershell
python tools/parameter_id/download_log_ble.py `
  -o artifacts/run-01.twlog

python tools/parameter_id/decode_twlog.py `
  artifacts/run-01.twlog `
  -o artifacts/run-01.csv
```

The `.twlog` file is versioned binary data with exact firmware timestamps. The decoder verifies magic, structure sizes, total payload length, and CRC before generating CSV.

## Untethered BLE diagnostic acquisition

The native NimBLE service remains available for low-rate diagnostics and historical acquisition tools:

```text
Device  TriWhirl
Service 54f10000-8f4d-4f3a-b691-54524957484c
RX      54f10001-8f4d-4f3a-b691-54524957484c
TX      54f10002-8f4d-4f3a-b691-54524957484c
```

`acquire_ble.py`, `body_free_ble.py`, `body_local_ble.py`, and `body_active_ble.py` are retained because their existing datasets remain useful evidence. They must not be treated as the final realtime timing path for rapid near-upright control.

For nonzero-`Vq` BLE work, the tools automatically reapply `artifacts/motor-config.json` after a battery boot. The commissioned configuration is therefore reused without another calibration. Negative segment values should be passed with `=` in PowerShell, for example `--segment=-0.25:0.8`.

Use `--address <BLE-address-or-device-id>` only if name-based discovery is ambiguous; otherwise the default `TriWhirl` scan is sufficient.

## Autonomous swing identification history

`auto_swing_id_ble.py` proved that the reaction wheel can pump the untethered body from a resting rocking motion through the neighborhood of the upright vertices without a hand release. It also established why the control decision must move on-device: when the host attempted to change `Vq` at a vertex, the requested input often appeared in firmware telemetry only after the useful local window had passed.

The host script is therefore retained as diagnostic/research history rather than the final experiment engine. The next autonomous path is the native 1 kHz firmware state machine:

```text
pump -> approach -> critical local window -> probe/capture -> recover -> repeat
```

Vertex detection, pump/probe `Vq` decisions, critical-window markers, and TWLG records all execute on the ESP32. BLE only configures/starts the run, reports coarse status, and downloads the completed log.

Historical host-side artifacts still have value:

- `*-raw.csv` preserves global rocking trajectories;
- local CSVs preserve prior A/B/C windows;
- JSON sidecars preserve host experiment parameters/provenance.

This identification infrastructure does not by itself close the final swing-up-controller work. The production hybrid controller still requires the global rocking/contact model, explicit wheel-speed limits, capture supervision, and recovery behavior.

## UART acquisition

`acquire.py` is retained for tethered motor/actuator work where the USB cable does not affect the experiment. It drives only the `Vq` values explicitly supplied on the command line and does not invent excitation amplitudes or hardware safety thresholds.

Example profile:

```powershell
python tools/parameter_id/acquire.py COM28 `
  --segment=0.25:0.8 `
  --segment=0:0.4 `
  --segment=-0.25:0.8 `
  --segment=0:0.4 `
  --repeat 4 `
  -o logs/local-id.csv
```

A segment is `Vq_volts:duration_seconds`. The UART tool:

- opens the existing CH340 UART;
- commands the motor stopped before acquisition;
- checks that a motor electrical configuration exists;
- automatically reloads the last commissioned electrical configuration from `artifacts/motor-config.json` when firmware has restarted without one;
- optionally runs the existing firmware calibration with `--auto-calibrate` when no reusable configuration exists, then saves pole pairs, sensor direction, and electrical offset for later runs;
- waits for valid attitude and wheel-rate telemetry;
- enables telemetry and executes the requested `Vq` profile;
- aborts on any firmware `FAULT` or nonzero `fault_mask`;
- always sends `motor stop` and disables telemetry on exit;
- writes schema-v2 CSV plus a JSON sidecar containing the exact excitation profile, motor configuration, and run metadata.

The CSV adds a `phase` column but otherwise preserves the normal telemetry field names, so it can be consumed directly by `local_fit.py`.

## Preliminary flat-table actuator fit

`local_fit.py` consumes telemetry schema v2 and fits a preliminary continuous-time local model using the measured firmware state and commanded `Vq`:

```text
theta_ddot = a1*theta + a2*theta_rate + a3*wheel_rate + b1*Vq + c1
wheel_accel = a4*theta + a5*theta_rate + a6*wheel_rate + b2*Vq + c2
```

Only rows with `fault_mask=0`, `attitude_ok=1`, and `vel_valid=1` are used. Optional angle and voltage bounds can further restrict the fit to a chosen local operating region.

Example:

```powershell
python tools/parameter_id/local_fit.py logs/local-id.csv --max-abs-theta 0.25 --max-abs-vq 1.0 -o artifacts/local-fit.json
```

The fitter estimates derivatives from held-input local linear slopes rather than adjacent-sample differences. It reports coefficient uncertainty, RMSE, R², matrix rank, raw and normalized conditioning, singular values, sample period, and `Vq` coefficient significance.

A successful regression is not by itself a validated plant model; excitation quality, experiment posture, contact vertex, and physical consistency remain part of the model evidence.
