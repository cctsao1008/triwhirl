#!/usr/bin/env python3
"""Fit small-angle passive body dynamics from body_local_ble.py output.

The fused attitude angle is useful for long-term absolute orientation, but during
fast departure its accelerometer correction means d(theta_fused)/dt is not
identical to the corrected gyro rate. For local plant identification we need a
kinematically consistent state pair, so this fitter reconstructs the short-horizon
angle error by integrating theta_rate_rad_s from the last held sample before
release.

Nominal regression:
    theta_ddot = a_theta * theta_error_gyro
                + a_rate * theta_rate
                + a_wheel * wheel_rate
                + bias

The tool also reports the directly observable unstable modal growth relation
    theta_ddot ~= lambda_u * theta_rate + c
which remains meaningful when passive release trajectories excite mostly one
unstable mode and the individual A-row coefficients are not separately
identifiable.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit local upright body dynamics.")
    parser.add_argument("input", type=Path)
    parser.add_argument("--derivative-window", type=int, default=2,
                        help="samples on each side for local rate-slope derivative")
    parser.add_argument("--max-angle-deg", type=float, default=8.0,
                        help="maximum gyro-integrated |theta error| used in the fit")
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def angle_diff_scalar(angle: float, reference: float) -> float:
    return math.atan2(math.sin(angle - reference), math.cos(angle - reference))


def local_slope(t: np.ndarray, y: np.ndarray, center: int, half: int) -> float:
    lo = center - half
    hi = center + half + 1
    tt = t[lo:hi]
    yy = y[lo:hi]
    tc = tt - np.mean(tt)
    denom = float(np.dot(tc, tc))
    if denom <= 0.0:
        return float("nan")
    return float(np.dot(tc, yy - np.mean(yy)) / denom)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def normalized_condition_number(x: np.ndarray) -> float:
    scale = np.std(x, axis=0)
    keep = scale > 1.0e-12
    if not np.any(keep):
        return float("inf")
    xn = (x[:, keep] - np.mean(x[:, keep], axis=0)) / scale[keep]
    return float(np.linalg.cond(xn))


def ols(x: np.ndarray, y: np.ndarray):
    beta, _residuals, rank, singular = np.linalg.lstsq(x, y, rcond=None)
    y_hat = x @ beta
    resid = y - y_hat
    sse = float(np.dot(resid, resid))
    sst = float(np.dot(y - np.mean(y), y - np.mean(y)))
    r2 = 1.0 - sse / sst if sst > 0.0 else 0.0
    rmse = float(np.sqrt(np.mean(np.square(resid))))
    dof = len(y) - int(rank)
    if dof > 0 and int(rank) == x.shape[1]:
        covariance = (sse / dof) * np.linalg.inv(x.T @ x)
        stderr = np.sqrt(np.diag(covariance))
    else:
        stderr = np.full(x.shape[1], np.nan)
    return beta, stderr, int(rank), singular, r2, rmse


def integrate_gyro_error(trial_rows: list[dict[str, str]]):
    rows = sorted(trial_rows, key=lambda row: int(row["t_us"]))
    hold_indices = [i for i, row in enumerate(rows) if row.get("phase") == "hold"]
    if not hold_indices:
        raise RuntimeError("trial has no hold rows")
    start = hold_indices[-1]
    rows = rows[start:]
    t = np.asarray([float(row["t_us"]) * 1.0e-6 for row in rows])
    rate = np.asarray([float(row["theta_rate_rad_s"]) for row in rows])
    theta_ref = float(rows[0]["theta_ref_rad"])
    initial_error = angle_diff_scalar(float(rows[0]["theta_rad"]), theta_ref)
    error = np.zeros(len(rows), dtype=float)
    error[0] = initial_error
    for i in range(1, len(rows)):
        dt = t[i] - t[i - 1]
        if dt <= 0.0:
            error[i] = error[i - 1]
        else:
            error[i] = error[i - 1] + 0.5 * (rate[i] + rate[i - 1]) * dt
    return rows, t, rate, error


def main() -> int:
    args = parse_args()
    if args.derivative_window < 1:
        raise RuntimeError("--derivative-window must be >= 1")

    rows = read_rows(args.input)
    trials = sorted({int(row["trial"]) for row in rows if row.get("phase") == "local"})
    if not trials:
        raise RuntimeError("no phase=local rows found")

    regressors: list[list[float]] = []
    targets: list[float] = []
    sample_trial: list[int] = []
    per_trial: list[dict[str, object]] = []
    max_angle = math.radians(args.max_angle_deg)

    for trial in trials:
        trial_rows = [row for row in rows if int(row["trial"]) == trial]
        seq, t, rate, gyro_error = integrate_gyro_error(trial_rows)
        wheel = np.asarray([float(row["vel_rad_s"]) for row in seq])
        theta = np.asarray([float(row["theta_rad"]) for row in seq])
        theta_ref = float(seq[0]["theta_ref_rad"])

        used = 0
        fused_vs_gyro: list[float] = []
        used_errors: list[float] = []
        for i, row in enumerate(seq):
            if row.get("phase") != "local":
                continue
            if i < args.derivative_window or i + args.derivative_window >= len(seq):
                continue
            if abs(gyro_error[i]) > max_angle:
                continue
            window = t[i-args.derivative_window:i+args.derivative_window+1]
            if not np.all(np.diff(window) > 0.0):
                continue
            accel = local_slope(t, rate, i, args.derivative_window)
            if not np.isfinite(accel):
                continue
            regressors.append([gyro_error[i], rate[i], wheel[i], 1.0])
            targets.append(accel)
            sample_trial.append(trial)
            used_errors.append(float(gyro_error[i]))
            fused_error = angle_diff_scalar(theta[i], theta_ref)
            fused_vs_gyro.append(fused_error - gyro_error[i])
            used += 1

        local_indices = [i for i, row in enumerate(seq) if row.get("phase") == "local"]
        local_gyro_max = (
            math.degrees(max(abs(float(gyro_error[i])) for i in local_indices))
            if local_indices else 0.0
        )
        per_trial.append({
            "trial": trial,
            "local_rows": len(local_indices),
            "used": used,
            "theta_ref_rad": theta_ref,
            "theta_ref_deg": math.degrees(theta_ref),
            "max_abs_theta_error_gyro_deg": local_gyro_max,
            "fused_minus_gyro_error_rmse_rad": (
                float(np.sqrt(np.mean(np.square(fused_vs_gyro))))
                if fused_vs_gyro else None
            ),
            "used_direction": (
                "positive" if used_errors and min(used_errors) > 0.0
                else "negative" if used_errors and max(used_errors) < 0.0
                else "mixed" if used_errors else "none"
            ),
        })

    if len(targets) < 8:
        raise RuntimeError(
            f"only {len(targets)} usable dynamic samples; capture more local departures"
        )

    x = np.asarray(regressors, dtype=float)
    y = np.asarray(targets, dtype=float)
    beta, stderr, rank, singular, r2, rmse = ols(x, y)

    names = ["theta_error_gyro_rad", "theta_rate_rad_s", "wheel_rate_rad_s", "bias"]
    coefficients = {
        name: {
            "value": float(beta[i]),
            "std_error": None if not np.isfinite(stderr[i]) else float(stderr[i]),
            "abs_over_std_error": (
                None if not np.isfinite(stderr[i]) or stderr[i] == 0.0
                else float(abs(beta[i]) / stderr[i])
            ),
        }
        for i, name in enumerate(names)
    }

    # Directly observable unstable modal growth. This does not pretend to
    # separate stiffness and damping when all passive releases lie on the same
    # unstable eigenmode.
    modal_x = np.column_stack((x[:, 1], np.ones(len(x))))
    modal_beta, modal_se, _modal_rank, _modal_singular, modal_r2, modal_rmse = ols(
        modal_x, y
    )
    rate = x[:, 1]
    lambda_origin = float(np.dot(rate, y) / np.dot(rate, rate))
    origin_resid = y - lambda_origin * rate
    lambda_origin_se = float(
        math.sqrt(float(np.dot(origin_resid, origin_resid)) / (len(y) - 1)
                  / float(np.dot(rate, rate)))
    )

    error_rate_corr = float(np.corrcoef(x[:, 0], x[:, 1])[0, 1])
    positive = int(np.count_nonzero(x[:, 0] > 0.0))
    negative = int(np.count_nonzero(x[:, 0] < 0.0))
    reasons: list[str] = []
    theta_sig = coefficients["theta_error_gyro_rad"]["abs_over_std_error"]
    if positive == 0 or negative == 0:
        reasons.append("all usable departures have the same theta-error sign")
    if abs(error_rate_corr) >= 0.85:
        reasons.append("theta error and theta rate are strongly modal/collinear")
    if theta_sig is None or theta_sig < 2.0:
        reasons.append("theta stiffness coefficient is not statistically separated at 2-sigma")
    full_state_row_accepted = not reasons

    payload = {
        "format": "triwhirl-body-local-fit-v2",
        "input": str(args.input),
        "angle_state": "gyro-integrated from last held sample",
        "model": (
            "theta_ddot = a_theta*theta_error_gyro + a_rate*theta_rate "
            "+ a_wheel*wheel_rate + bias"
        ),
        "derivative_window_each_side": args.derivative_window,
        "max_angle_deg": args.max_angle_deg,
        "trials": per_trial,
        "sample_count": int(len(y)),
        "rank": rank,
        "condition_number_raw": float(np.linalg.cond(x)),
        "condition_number_normalized": normalized_condition_number(x[:, :3]),
        "rmse_rad_s2": rmse,
        "r_squared": float(r2),
        "singular_values": [float(value) for value in singular],
        "coefficients": coefficients,
        "state_row_candidate": {
            "theta_error": float(beta[0]),
            "theta_rate": float(beta[1]),
            "wheel_rate": float(beta[2]),
        },
        "bias_rad_s2": float(beta[3]),
        "modal_growth": {
            "model": "theta_ddot = lambda_u*theta_rate + intercept",
            "lambda_u_per_s": float(modal_beta[0]),
            "lambda_u_std_error_per_s": (
                None if not np.isfinite(modal_se[0]) else float(modal_se[0])
            ),
            "intercept_rad_s2": float(modal_beta[1]),
            "r_squared": float(modal_r2),
            "rmse_rad_s2": float(modal_rmse),
            "through_origin_lambda_u_per_s": lambda_origin,
            "through_origin_std_error_per_s": lambda_origin_se,
            "e_folding_time_s": float(1.0 / lambda_origin) if lambda_origin > 0.0 else None,
        },
        "identifiability": {
            "full_state_row_accepted": full_state_row_accepted,
            "theta_error_rate_correlation": error_rate_corr,
            "positive_theta_error_samples": positive,
            "negative_theta_error_samples": negative,
            "reasons": reasons,
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    print(
        f"samples={len(y)} rank={rank} R2={r2:.4f} RMSE={rmse:.4f} rad/s^2 "
        f"cond_norm={payload['condition_number_normalized']:.2f}"
    )
    for name in names:
        item = coefficients[name]
        se = item["std_error"]
        ratio = item["abs_over_std_error"]
        print(
            f"{name}: {item['value']:.6g}"
            + (f" +/- {se:.6g}" if se is not None else "")
            + (f"  |coef|/SE={ratio:.2f}" if ratio is not None else "")
        )
    mg = payload["modal_growth"]
    print(
        f"unstable modal growth: lambda_u={mg['through_origin_lambda_u_per_s']:.4f} "
        f"+/- {mg['through_origin_std_error_per_s']:.4f} 1/s, "
        f"tau={mg['e_folding_time_s']:.4f} s"
    )
    if reasons:
        print("full A-row not accepted: " + "; ".join(reasons))
    else:
        print("full A-row accepted by current identifiability checks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
