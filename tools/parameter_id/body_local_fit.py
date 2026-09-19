#!/usr/bin/env python3
"""Fit the small-angle passive body dynamics from body_local_ble.py output.

Model:
    theta_ddot = a_theta * theta_error
                + a_rate * theta_rate
                + a_wheel * wheel_rate
                + bias

The input is intentionally limited to rows marked phase=local. Held/armed rows
contain hand forces and are not used for the dynamic regression.
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
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def angle_diff(angle: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angle - reference), np.cos(angle - reference))


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
    if xn.shape[1] == 0:
        return float("inf")
    return float(np.linalg.cond(xn))


def main() -> int:
    args = parse_args()
    if args.derivative_window < 1:
        raise RuntimeError("--derivative-window must be >= 1")

    rows = read_rows(args.input)
    local = [row for row in rows if row.get("phase") == "local"]
    if not local:
        raise RuntimeError("no phase=local rows found")

    trials = sorted({int(row["trial"]) for row in local})
    regressors: list[list[float]] = []
    targets: list[float] = []
    per_trial: list[dict[str, object]] = []
    max_angle = math.radians(args.max_angle_deg)

    for trial in trials:
        tr = [row for row in local if int(row["trial"]) == trial]
        if len(tr) < 2 * args.derivative_window + 3:
            per_trial.append({
                "trial": trial,
                "rows": len(tr),
                "used": 0,
                "reason": "too_few_rows",
            })
            continue

        t = np.asarray([float(row["t_us"]) * 1.0e-6 for row in tr])
        theta = np.asarray([float(row["theta_rad"]) for row in tr])
        theta_ref = np.asarray([float(row["theta_ref_rad"]) for row in tr])
        rate = np.asarray([float(row["theta_rate_rad_s"]) for row in tr])
        wheel = np.asarray([float(row["vel_rad_s"]) for row in tr])
        error = angle_diff(theta, theta_ref)

        trial_used = 0
        kin_errors: list[float] = []
        for i in range(args.derivative_window, len(tr) - args.derivative_window):
            if abs(error[i]) > max_angle:
                continue
            if not np.all(np.diff(t[i-args.derivative_window:i+args.derivative_window+1]) > 0.0):
                continue
            accel = local_slope(t, rate, i, args.derivative_window)
            theta_dot_from_angle = local_slope(t, theta, i, args.derivative_window)
            if not np.isfinite(accel):
                continue
            regressors.append([error[i], rate[i], wheel[i], 1.0])
            targets.append(accel)
            if np.isfinite(theta_dot_from_angle):
                kin_errors.append(theta_dot_from_angle - rate[i])
            trial_used += 1

        per_trial.append({
            "trial": trial,
            "rows": len(tr),
            "used": trial_used,
            "theta_ref_rad": float(np.median(theta_ref)),
            "theta_ref_deg": math.degrees(float(np.median(theta_ref))),
            "max_abs_theta_error_deg": math.degrees(float(np.max(np.abs(error)))),
            "kinematic_rate_rmse_rad_s": (
                float(np.sqrt(np.mean(np.square(kin_errors)))) if kin_errors else None
            ),
        })

    if len(targets) < 8:
        raise RuntimeError(
            f"only {len(targets)} usable dynamic samples; capture more local departures"
        )

    x = np.asarray(regressors, dtype=float)
    y = np.asarray(targets, dtype=float)
    beta, residuals, rank, singular = np.linalg.lstsq(x, y, rcond=None)
    y_hat = x @ beta
    resid = y - y_hat
    sse = float(np.dot(resid, resid))
    sst = float(np.dot(y - np.mean(y), y - np.mean(y)))
    r2 = 1.0 - sse / sst if sst > 0.0 else 0.0
    rmse = float(np.sqrt(np.mean(np.square(resid))))

    dof = len(y) - int(rank)
    if dof > 0 and int(rank) == x.shape[1]:
        sigma2 = sse / dof
        covariance = sigma2 * np.linalg.inv(x.T @ x)
        stderr = np.sqrt(np.diag(covariance))
    else:
        stderr = np.full(x.shape[1], np.nan)

    names = ["theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s", "bias"]
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

    payload = {
        "format": "triwhirl-body-local-fit-v1",
        "input": str(args.input),
        "model": (
            "theta_ddot = a_theta*theta_error + a_rate*theta_rate "
            "+ a_wheel*wheel_rate + bias"
        ),
        "derivative_window_each_side": args.derivative_window,
        "max_angle_deg": args.max_angle_deg,
        "trials": per_trial,
        "sample_count": int(len(y)),
        "rank": int(rank),
        "condition_number_raw": float(np.linalg.cond(x)),
        "condition_number_normalized": normalized_condition_number(x[:, :3]),
        "rmse_rad_s2": rmse,
        "r_squared": float(r2),
        "singular_values": [float(value) for value in singular],
        "coefficients": coefficients,
        "state_row": {
            "theta_error": float(beta[0]),
            "theta_rate": float(beta[1]),
            "wheel_rate": float(beta[2]),
        },
        "bias_rad_s2": float(beta[3]),
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
