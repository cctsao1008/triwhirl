# Local parameter identification

TriWhirl keeps acquisition and model fitting separate: the firmware owns motor/safety limits, `acquire.py` records an explicit commanded experiment, and `local_fit.py` estimates a preliminary local model from the resulting telemetry.

## Acquire a run

`acquire.py` drives only the `Vq` values explicitly supplied on the command line. It does not invent excitation amplitudes or hardware safety thresholds.

Example profile:

```powershell
python tools/parameter_id/acquire.py COM28 `
  --segment 0.25:0.8 `
  --segment 0:0.4 `
  --segment -0.25:0.8 `
  --segment 0:0.4 `
  --repeat 4 `
  -o logs/local-id.csv
```

A segment is `Vq_volts:duration_seconds`. The tool:

- opens the existing CH340 UART;
- commands the motor stopped before acquisition;
- checks that a motor electrical configuration exists;
- optionally runs the existing firmware calibration with `--auto-calibrate` if configuration is absent;
- waits for valid attitude and wheel-rate telemetry;
- enables telemetry and executes the requested `Vq` profile;
- aborts on any firmware `FAULT` or nonzero `fault_mask`;
- always sends `motor stop` and `telemetry off` on exit;
- writes schema-v2 CSV plus a JSON sidecar containing the exact excitation profile and run metadata.

The CSV adds a `phase` column but otherwise preserves the normal telemetry field names, so it can be consumed directly by `local_fit.py`.

If the motor has not yet been commissioned after boot, the same run can request the existing firmware calibration:

```powershell
python tools/parameter_id/acquire.py COM28 --auto-calibrate --segment 0.25:0.8 --segment 0:0.4 -o logs/local-id.csv
```

The operator still chooses the actual `Vq` profile. Firmware remains authoritative for actuation limits and latched faults.

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

The fitter requires NumPy and reports coefficients, coefficient standard errors, RMSE, R², matrix rank, condition number, singular values, sample period, and the filters applied.

The result is not treated as a validated plant merely because the regression runs. A controller-quality model still requires controlled near-upright experiment logs with sufficient excitation, repeated operating conditions, and residual review.
