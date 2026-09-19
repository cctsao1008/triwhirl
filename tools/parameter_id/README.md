# Local parameter identification

TriWhirl keeps acquisition and model fitting separate: the firmware owns motor/safety limits, the host acquisition tools record an explicit commanded experiment, and `local_fit.py` estimates a preliminary local model from the resulting telemetry.

## Python dependencies

Install the host-side dependencies into the active Python environment once:

```powershell
python -m pip install -r tools/parameter_id/requirements.txt
```

This installs NumPy for fitting, pySerial for UART acquisition, and Bleak for untethered BLE acquisition.

## Untethered BLE acquisition

For body-motion identification the USB cable must not mechanically disturb the TriWhirl body. The existing native NimBLE firmware already exposes the same command/telemetry protocol used by UART, so `acquire_ble.py` records the experiment while the unit runs from its battery.

The BLE service is the same one used by the WebUI:

```text
Device  TriWhirl
Service 54f10000-8f4d-4f3a-b691-54524957484c
RX      54f10001-8f4d-4f3a-b691-54524957484c
TX      54f10002-8f4d-4f3a-b691-54524957484c
```

A zero-actuation free-body capture does not require a motor electrical configuration:

```powershell
python tools/parameter_id/acquire_ble.py `
  --segment=0:8 `
  --pre-roll 0 `
  --post-roll 0 `
  -o artifacts/body-free-01.csv
```

The tool scans for `TriWhirl`, connects to its native GATT service, subscribes to telemetry, waits for valid attitude and wheel-rate state, records schema-v2 CSV, and always sends `motor stop` / `telemetry off` before disconnecting when the connection remains available.

For a later nonzero-`Vq` BLE experiment, the tool automatically reapplies `artifacts/motor-config.json` after a battery boot. The same commissioned motor configuration used by UART is therefore reused without another calibration. Negative segment values should be passed with `=` in PowerShell, for example `--segment=-0.25:0.8`.

Use `--address <BLE-address-or-device-id>` only if name-based discovery is ambiguous; otherwise the default `TriWhirl` scan is sufficient.

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
- optionally runs the existing firmware calibration with `--auto-calibrate` when no reusable configuration exists, then saves the resulting pole pairs, sensor direction, and electrical offset for later runs;
- waits for valid attitude and wheel-rate telemetry;
- enables telemetry and executes the requested `Vq` profile;
- aborts on any firmware `FAULT` or nonzero `fault_mask`;
- always sends `motor stop` and `telemetry off` on exit;
- writes schema-v2 CSV plus a JSON sidecar containing the exact excitation profile, motor configuration, and run metadata.

The CSV adds a `phase` column but otherwise preserves the normal telemetry field names, so it can be consumed directly by `local_fit.py`.

## Fit the local model

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

A successful regression is not by itself a validated plant model; excitation quality, experiment posture, and physical consistency remain part of the model evidence.
