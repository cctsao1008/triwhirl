# Local parameter identification

`local_fit.py` consumes telemetry schema v2 and fits a preliminary continuous-time local model using the measured firmware state and commanded `Vq`:

```text
theta_ddot = a1*theta + a2*theta_rate + a3*wheel_rate + b1*Vq + c1
wheel_accel = a4*theta + a5*theta_rate + a6*wheel_rate + b2*Vq + c2
```

Only rows with `fault_mask=0`, `attitude_ok=1`, and `vel_valid=1` are used. Optional angle and voltage bounds can further restrict the fit to a chosen local operating region.

Example:

```powershell
python tools/parameter_id/local_fit.py logs/run.csv --max-abs-theta 0.25 --max-abs-vq 1.0 -o artifacts/local-fit.json
```

The script requires NumPy and reports coefficients, RMSE, R², matrix rank, sample period, and the filters applied.

The result is not treated as a validated plant merely because the regression runs. A controller-quality model still requires controlled near-upright experiment logs with sufficient excitation, repeated operating conditions, and residual review.
