#!/usr/bin/env python3
"""Fit the actively excited near-upright TriWhirl local model.

Body model:
    theta_ddot = a_theta * theta_error_gyro
               + a_rate  * theta_rate
               + a_wheel * wheel_rate
               + b_vq    * vq
               + bias

Wheel model:
    wheel_accel = c_theta * theta_error_gyro
                + c_rate  * theta_rate
                + c_wheel * wheel_rate
                + d_vq    * vq
                + bias

`theta_error_gyro` is integrated from the calibrated gyro starting at the last
armed sample before release. This keeps angle, rate, and acceleration
kinematically consistent. The measured firmware telemetry `vq_v`, not the
planned host command, is the identification input.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit active near-upright plant dynamics.")
    parser.add_argument("input", type=Path)
    parser.add_argument("--derivative-window", type=int, default=1)
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


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


def normalized_condition_number(x: np.ndarray) -> float:
    # Normalize physical regressors only; the intercept is intentionally omitted.
    if x.shape[1] <= 1:
        return float("inf")
    physical = x[:, :-1]
    scale = np.std(physical, axis=0)
    keep = scale > 1.0e-12
    if not np.any(keep):
        return float("inf")
    xn = (physical[:, keep] - np.mean(physical[:, keep], axis=0)) / scale[keep]
    return float(np.linalg.cond(xn))


def fit_equation(x: np.ndarray, y: np.ndarray, names: list[str]) -> dict[str, object]:
    beta, _residuals, rank, singular = np.linalg.lstsq(x, y, rcond=None)
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

    coefficients: dict[str, object] = {}
    for i, name in enumerate(names):
        se = None if not np.isfinite(stderr[i]) else float(stderr[i])
        ratio = None if se is None or se == 0.0 else float(abs(beta[i]) / se)
        coefficients[name] = {
            "value": float(beta[i]),
            "std_error": se,
            "abs_over_std_error": ratio,
        }

    return {
        "rank": int(rank),
        "rmse": rmse,
        "r_squared": float(r2),
        "singular_values": [float(v) for v in singular],
        "coefficients": coefficients,
    }


def main() -> int:
    args = parse_args()
    if args.derivative_window < 1:
        raise RuntimeError("--derivative-window must be >= 1")

    rows = read_rows(args.input)
    if not rows:
        raise RuntimeError("input CSV is empty")

    trial_ids = sorted({int(r["trial"]) for r in rows if r.get("trial")})
    max_angle = math.radians(args.max_angle_deg)
    regressors: list[list[float]] = []
    body_targets: list[float] = []
    wheel_targets: list[float] = []
    per_trial: list[dict[str, object]] = []
    rejected_transition = 0
    rejected_angle = 0

    for trial in trial_ids:
        tr = [r for r in rows if int(r.get("trial", "0")) == trial]
        tr.sort(key=lambda r: int(r["t_us"]))
        dynamic_indices = [
            i for i, r in enumerate(tr)
            if r.get("phase") in ("active", "zero_vector")
        ]
        if not dynamic_indices:
            per_trial.append({"trial": trial, "used": 0, "reason": "no_dynamic_rows"})
            continue

        first_dynamic = dynamic_indices[0]
        armed_before = [i for i in range(first_dynamic) if tr[i].get("phase") == "armed"]
        anchor_i = armed_before[-1] if armed_before else max(0, first_dynamic - 1)
        dyn = tr[anchor_i: dynamic_indices[-1] + 1]

        t = np.asarray([float(r["t_us"]) * 1.0e-6 for r in dyn])
        rate = np.asarray([float(r["theta_rate_rad_s"]) for r in dyn])
        wheel = np.asarray([float(r["vel_rad_s"]) for r in dyn])
        vq = np.asarray([float(r["vq_v"]) for r in dyn])
        theta = np.asarray([float(r["theta_rad"]) for r in dyn])
        theta_ref = float(dyn[0]["theta_ref_rad"])

        error = np.empty(len(dyn), dtype=float)
        error[0] = angle_diff(theta[0], theta_ref)
        for i in range(1, len(dyn)):
            dt = t[i] - t[i - 1]
            if dt <= 0.0:
                error[i] = error[i - 1]
            else:
                error[i] = error[i - 1] + 0.5 * (rate[i - 1] + rate[i]) * dt

        used = 0
        active_used = 0
        zero_used = 0
        for i in range(args.derivative_window, len(dyn) - args.derivative_window):
            if dyn[i].get("phase") not in ("active", "zero_vector"):
                continue
            lo = i - args.derivative_window
            hi = i + args.derivative_window + 1
            if not np.all(np.diff(t[lo:hi]) > 0.0):
                continue
            if abs(error[i]) > max_angle:
                rejected_angle += 1
                continue
            # Do not estimate an acceleration across a Vq step. The center's
            # telemetry Vq is authoritative once the whole derivative window is held.
            if float(np.max(vq[lo:hi]) - np.min(vq[lo:hi])) > 1.0e-6:
                rejected_transition += 1
                continue

            body_accel = local_slope(t, rate, i, args.derivative_window)
            wheel_accel = local_slope(t, wheel, i, args.derivative_window)
            if not np.isfinite(body_accel) or not np.isfinite(wheel_accel):
                continue

            regressors.append([error[i], rate[i], wheel[i], vq[i], 1.0])
            body_targets.append(body_accel)
            wheel_targets.append(wheel_accel)
            used += 1
            if abs(vq[i]) > 1.0e-9:
                active_used += 1
            else:
                zero_used += 1

        per_trial.append({
            "trial": trial,
            "theta_ref_rad": theta_ref,
            "theta_ref_deg": math.degrees(theta_ref),
            "planned_vq_v": float(tr[first_dynamic].get("planned_vq_v", "nan")),
            "dynamic_rows": len(dynamic_indices),
            "used": used,
            "active_used": active_used,
            "zero_vector_used": zero_used,
            "theta_error_min_deg": math.degrees(float(np.min(error))),
            "theta_error_max_deg": math.degrees(float(np.max(error))),
            "measured_vq_min_v": float(np.min(vq)),
            "measured_vq_max_v": float(np.max(vq)),
        })

    if len(regressors) < 12:
        raise RuntimeError(
            f"only {len(regressors)} usable dynamic samples; active capture is insufficient"
        )

    x = np.asarray(regressors, dtype=float)
    y_body = np.asarray(body_targets, dtype=float)
    y_wheel = np.asarray(wheel_targets, dtype=float)
    names = [
        "theta_error_gyro_rad", "theta_rate_rad_s", "wheel_rate_rad_s",
        "vq_v", "bias",
    ]
    body_fit = fit_equation(x, y_body, names)
    wheel_fit = fit_equation(x, y_wheel, names)

    vq_values = x[:, 3]
    theta_values = x[:, 0]
    payload = {
        "format": "triwhirl-body-active-fit-v1",
        "input": str(args.input),
        "model": {
            "body": "theta_ddot = a_theta*theta_error_gyro + a_rate*theta_rate + a_wheel*wheel_rate + b_vq*vq + bias",
            "wheel": "wheel_accel = c_theta*theta_error_gyro + c_rate*theta_rate + c_wheel*wheel_rate + d_vq*vq + bias",
        },
        "sample_count": int(len(x)),
        "derivative_window_each_side": args.derivative_window,
        "max_angle_deg": args.max_angle_deg,
        "condition_number_raw": float(np.linalg.cond(x)),
        "condition_number_normalized": normalized_condition_number(x),
        "input_coverage": {
            "vq_min_v": float(np.min(vq_values)),
            "vq_max_v": float(np.max(vq_values)),
            "positive_vq_samples": int(np.sum(vq_values > 1.0e-9)),
            "negative_vq_samples": int(np.sum(vq_values < -1.0e-9)),
            "zero_vq_samples": int(np.sum(np.abs(vq_values) <= 1.0e-9)),
            "positive_theta_error_samples": int(np.sum(theta_values > 0.0)),
            "negative_theta_error_samples": int(np.sum(theta_values < 0.0)),
        },
        "rejected_windows": {
            "vq_transition": rejected_transition,
            "angle_limit": rejected_angle,
        },
        "trials": per_trial,
        "body_equation": body_fit,
        "wheel_equation": wheel_fit,
        "candidate_state_row": {
            "theta_error": body_fit["coefficients"]["theta_error_gyro_rad"]["value"],
            "theta_rate": body_fit["coefficients"]["theta_rate_rad_s"]["value"],
            "wheel_rate": body_fit["coefficients"]["wheel_rate_rad_s"]["value"],
        },
        "candidate_body_input_gain_vq": body_fit["coefficients"]["vq_v"]["value"],
        "candidate_wheel_state_row": {
            "theta_error": wheel_fit["coefficients"]["theta_error_gyro_rad"]["value"],
            "theta_rate": wheel_fit["coefficients"]["theta_rate_rad_s"]["value"],
            "wheel_rate": wheel_fit["coefficients"]["wheel_rate_rad_s"]["value"],
        },
        "candidate_wheel_input_gain_vq": wheel_fit["coefficients"]["vq_v"]["value"],
        "interpretation": (
            "Candidate local A/B evidence only. Accept coefficients for controller synthesis "
            "only after reviewing sign coverage, coefficient uncertainty, conditioning, and residual quality."
        ),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print(args.output)
    print(
        f"samples={len(x)} cond_norm={payload['condition_number_normalized']:.2f} "
        f"Vq=[{payload['input_coverage']['vq_min_v']:.3f}, "
        f"{payload['input_coverage']['vq_max_v']:.3f}] V"
    )
    for label, fit in (("body", body_fit), ("wheel", wheel_fit)):
        print(
            f"{label}: rank={fit['rank']} R2={fit['r_squared']:.4f} "
            f"RMSE={fit['rmse']:.4f}"
        )
        for name in names:
            item = fit["coefficients"][name]
            se = item["std_error"]
            ratio = item["abs_over_std_error"]
            print(
                f"  {name}: {item['value']:.6g}"
                + (f" +/- {se:.6g}" if se is not None else "")
                + (f"  |coef|/SE={ratio:.2f}" if ratio is not None else "")
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
