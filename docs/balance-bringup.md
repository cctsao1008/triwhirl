# Closed-loop balance bring-up

This is the shortest supported path from firmware-owned identification to a guarded near-upright balance trial.

## 1. Build and flash the current firmware

The runtime now owns the full 1 kHz balance loop on ESP32 Core 1. UART/BLE remain supervisory only. Balance actuation is admitted only when motor configuration, encoder, IMU calibration, attitude validity, safety state, capture angle, and wheel speed are all valid.

## 2. Collect one firmware-owned swing-identification run

```bash
python tools/twtool.py id swing \
  -o artifacts/id/run01.twlog \
  --vertex-a-deg 68
```

A successful run writes a fit-ready file beside the TWLG binary, normally:

```text
artifacts/id/run01-active.csv
```

Do not synthesize a deployment controller from an aborted run or from a run with unusable probe windows.

## 3. Fit the plant and synthesize one robust H-infinity gain

```bash
python tools/twtool.py fit hinf \
  artifacts/id/run01-active.csv \
  --vertex-a-deg 68 \
  --capture-deg 6 \
  --fall-deg 24 \
  --output-dir artifacts/hinf/run01
```

The pipeline performs:

```text
active probe CSV
  -> active upright fit
  -> local linear model
  -> polytopic uncertainty model
  -> common-Lyapunov H-infinity synthesis
  -> validated balance config command
```

The controller artifact is:

```text
artifacts/hinf/run01/run01-hinf.json
```

and the generated firmware command is:

```text
artifacts/hinf/run01/run01-balance-command.txt
```

The deployment converter refuses artifacts that do not report all plant vertices stable.

## 4. Configure without starting

```bash
python tools/twtool.py control balance \
  artifacts/hinf/run01/run01-hinf.json \
  --motor-config artifacts/motor-config.json \
  --theta-reference-deg 68 \
  --capture-deg 6 \
  --fall-deg 24
```

This connection stops the motor, restores the calibrated motor configuration, calibrates the gyro while stationary, resets attitude from gravity, applies the H-infinity gain, and verifies `balance status` plus `fault status`. It does **not** start the motor unless `--start` is supplied.

## 5. Run the first bounded balance trial

Hold the unit close to the intended upright vertex and keep clear of the reaction wheel. Then run:

```bash
python tools/twtool.py control balance \
  artifacts/hinf/run01/run01-hinf.json \
  --motor-config artifacts/motor-config.json \
  --theta-reference-deg 68 \
  --capture-deg 6 \
  --fall-deg 24 \
  --start \
  --duration 5
```

Firmware rejects `balance start` unless the body is inside the capture window and the wheel is below its configured rate limit. During the trial the host polls `balance status`; if firmware leaves Balance or latches a fault, the host reports the failure. At the end of a finite trial the host sends `balance stop`.

Use `--duration 0` only after bounded trials are stable; it deliberately leaves Balance active after the BLE client disconnects.

## Controller convention

The offline and realtime implementations use the same state and sign convention:

```text
x = [theta_error, theta_rate, wheel_rate]^T
Vq = -K_inf x
```

`theta_error` is the shared 120-degree-periodic upright coordinate, so the same controller applies at all three physical Reuleaux vertices.

## What is still hardware-dependent

The repository intentionally does not ship a fabricated gain. A real `K_inf` requires a successful identification log from the actual unit. The first hardware trial should therefore proceed in this order: identify, synthesize, configure-only, bounded start, inspect faults/timing, then extend duration.
