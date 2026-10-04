#!/usr/bin/env python3
"""Fit vertex-agnostic near-upright TriWhirl dynamics from body-active CSV.

Current identification coordinate:
    x = [theta_error_rad, theta_rate_rad_s, wheel_rate_rad_s]^T
    theta_error = wrap(theta - theta_ref)
    u = measured firmware Vq_v

Each acquisition trial carries its own held `theta_ref_rad`. Absolute orientation
is provenance only; this fitter never classifies A/B/C physical vertices.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

REGRESSOR_NAMES = (
    "theta_error_rad",
    "theta_rate_rad_s",
    "wheel_rate_rad_s",
    "vq_v",
    "bias",
)
DYNAMIC_PHASES = {"active", "zero_vector"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit a shared vertex-agnostic local plant from body-active trials."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("--derivative-window", type=int, default=3)
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument("--min-trials", type=int, default=2)
    parser.add_argument("--min-active-samples-per-sign", type=int, default=6)
    parser.add_argument("--min-theta-samples-per-sign", type=int, default=4)
    parser.add_argument("--min-vq-coef-sigma", type=float, default=2.0)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def local_slope(time_s: np.ndarray, values: np.ndarray) -> float | None:
    centered_t = time_s - np.mean(time_s)
    denominator = float(centered_t @ centered_t)
    if denominator <= 0.0:
        return None
    centered_y = values - np.mean(values)
    return float(centered_t @ centered_y / denominator)


def normalized_condition_number(x: np.ndarray) -> float | None:
    if x.ndim != 2 or x.shape[1] < 2:
        return None
    physical = x[:, :-1]
    scale = np.std(physical, axis=0)
    keep = scale > 1.0e-12
    if not np.any(keep):
        return None
    normalized = (
        physical[:, keep] - np.mean(physical[:, keep], axis=0)
    ) / scale[keep]
    value = float(np.linalg.cond(normalized))
    return value if math.isfinite(value) else None


def fit_equation(x: np.ndarray, y: np.ndarray) -> dict[str, object]:
    beta, _residuals, rank, singular = np.linalg.lstsq(x, y, rcond=None)
    predicted = x @ beta
    residual = y - predicted
    sse = float(residual @ residual)
    centered = y - np.mean(y)
    sst = float(centered @ centered)
    r2 = 1.0 - sse / sst if sst > 0.0 else None
    rmse = math.sqrt(sse / len(y))

    stderr = np.full(x.shape[1], np.nan)
    dof = len(y) - int(rank)
    if dof > 0 and int(rank) == x.shape[1]:
        covariance = (sse / dof) * np.linalg.pinv(x.T @ x)
        stderr = np.sqrt(np.maximum(np.diag(covariance), 0.0))

    coefficients: dict[str, object] = {}
    for index, name in enumerate(REGRESSOR_NAMES):
        se = None if not math.isfinite(float(stderr[index])) else float(stderr[index])
        ratio = None
        if se is not None and se > 0.0:
            ratio = float(abs(beta[index]) / se)
        coefficients[name] = {
            "value": float(beta[index]),
            "std_error": se,
            "abs_over_std_error": ratio,
        }

    return {
        "rank": int(rank),
        "rmse": rmse,
        "r_squared": None if r2 is None else float(r2),
        "singular_values": [float(value) for value in singular],
        "coefficients": coefficients,
    }


def row_usable(row: dict[str, str]) -> bool:
    try:
        if row.get("schema_version", "") and int(row["schema_version"]) != 2:
            return False
        if int(row.get("fault_mask", "0"), 0) != 0:
            return False
        if row.get("attitude_ok", "1") and int(row["attitude_ok"]) != 1:
            return False
        if row.get("vel_valid", "1") and int(row["vel_valid"]) != 1:
            return False
        values = (
            float(row["t_us"]),
            float(row["theta_rad"]),
            float(row["theta_rate_rad_s"]),
            float(row["vel_rad_s"]),
            float(row["vq_v"]),
            float(row["theta_ref_rad"]),
        )
        return all(math.isfinite(value) for value in values)
    except (KeyError, TypeError, ValueError):
        return False


def extract_trial(
    trial: int,
    rows: list[dict[str, str]],
    half_window: int,
    max_angle_rad: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, object]]:
    rows = sorted(rows, key=lambda row: int(row["t_us"]))
    references = [
        float(row["theta_ref_rad"])
        for row in rows
        if row.get("theta_ref_rad", "") and row_usable(row)
    ]
    if not references:
        return (
            np.empty((0, len(REGRESSOR_NAMES))),
            np.empty(0),
            np.empty(0),
            {"trial": trial, "status": "missing_theta_ref", "used_samples": 0},
        )

    theta_ref = float(np.median(np.asarray(references, dtype=float)))
    time_s = np.asarray([float(row["t_us"]) * 1.0e-6 for row in rows], dtype=float)
    theta = np.asarray([float(row.get("theta_rad", "nan")) for row in rows], dtype=float)
    rate = np.asarray(
        [float(row.get("theta_rate_rad_s", "nan")) for row in rows], dtype=float
    )
    wheel = np.asarray([float(row.get("vel_rad_s", "nan")) for row in rows], dtype=float)
    vq = np.asarray([float(row.get("vq_v", "nan")) for row in rows], dtype=float)
    error = np.asarray([wrap_angle(value - theta_ref) for value in theta], dtype=float)
    phases = [row.get("phase", "") for row in rows]
    usable = [row_usable(row) for row in rows]

    x_rows: list[list[float]] = []
    body_targets: list[float] = []
    wheel_targets: list[float] = []
    rejected_transition = 0
    rejected_invalid = 0
    rejected_angle = 0
    rejected_time = 0

    for center in range(half_window, len(rows) - half_window):
        if phases[center] not in DYNAMIC_PHASES:
            continue
        lo = center - half_window
        hi = center + half_window + 1
        if not all(usable[lo:hi]):
            rejected_invalid += 1
            continue
        if any(phases[index] != phases[center] for index in range(lo, hi)):
            rejected_transition += 1
            continue
        if np.any(np.diff(time_s[lo:hi]) <= 0.0):
            rejected_time += 1
            continue
        if float(np.max(vq[lo:hi]) - np.min(vq[lo:hi])) > 1.0e-6:
            rejected_transition += 1
            continue
        if abs(error[center]) > max_angle_rad:
            rejected_angle += 1
            continue

        body_accel = local_slope(time_s[lo:hi], rate[lo:hi])
        wheel_accel = local_slope(time_s[lo:hi], wheel[lo:hi])
        if body_accel is None or wheel_accel is None:
            rejected_time += 1
            continue

        x_rows.append(
            [error[center], rate[center], wheel[center], vq[center], 1.0]
        )
        body_targets.append(body_accel)
        wheel_targets.append(wheel_accel)

    x = np.asarray(x_rows, dtype=float)
    y_body = np.asarray(body_targets, dtype=float)
    y_wheel = np.asarray(wheel_targets, dtype=float)
    dynamic_error = [
        error[index]
        for index, phase in enumerate(phases)
        if phase in DYNAMIC_PHASES and math.isfinite(float(error[index]))
    ]
    dynamic_vq = [
        vq[index]
        for index, phase in enumerate(phases)
        if phase in DYNAMIC_PHASES and math.isfinite(float(vq[index]))
    ]
    meta = {
        "trial": trial,
        "status": "ok" if len(x_rows) else "no_usable_dynamic_windows",
        "theta_ref_rad": theta_ref,
        "theta_ref_deg": math.degrees(theta_ref),
        "used_samples": len(x_rows),
        "theta_error_min_deg": (
            math.degrees(float(min(dynamic_error))) if dynamic_error else None
        ),
        "theta_error_max_deg": (
            math.degrees(float(max(dynamic_error))) if dynamic_error else None
        ),
        "measured_vq_min_v": float(min(dynamic_vq)) if dynamic_vq else None,
        "measured_vq_max_v": float(max(dynamic_vq)) if dynamic_vq else None,
        "rejected_windows": {
            "input_or_phase_transition": rejected_transition,
            "invalid_telemetry": rejected_invalid,
            "angle_limit": rejected_angle,
            "time_or_derivative": rejected_time,
        },
    }
    return x, y_body, y_wheel, meta


def trial_fit(
    trial: int,
    x: np.ndarray,
    y_body: np.ndarray,
    y_wheel: np.ndarray,
) -> dict[str, object]:
    result: dict[str, object] = {
        "trial": trial,
        "sample_count": int(len(x)),
        "condition_number_normalized": normalized_condition_number(x)
        if len(x)
        else None,
    }
    if len(x) < len(REGRESSOR_NAMES) + 2:
        result["status"] = "insufficient_data"
        return result
    body = fit_equation(x, y_body)
    wheel = fit_equation(x, y_wheel)
    result["body_equation"] = body
    result["wheel_equation"] = wheel
    result["status"] = (
        "full_rank"
        if body["rank"] == len(REGRESSOR_NAMES)
        and wheel["rank"] == len(REGRESSOR_NAMES)
        else "rank_deficient"
    )
    return result


def coefficient_variation(trial_fits: list[dict[str, object]]) -> dict[str, object]:
    output: dict[str, object] = {}
    usable = [fit for fit in trial_fits if fit.get("status") == "full_rank"]
    for equation in ("body_equation", "wheel_equation"):
        equation_out: dict[str, object] = {}
        for name in REGRESSOR_NAMES:
            values: list[float] = []
            for fit in usable:
                try:
                    values.append(float(fit[equation]["coefficients"][name]["value"]))
                except (KeyError, TypeError, ValueError):
                    pass
            if values:
                equation_out[name] = {
                    "count": len(values),
                    "min": min(values),
                    "max": max(values),
                    "mean": sum(values) / len(values),
                }
        output[equation] = equation_out
    output["full_rank_trial_count"] = len(usable)
    return output


def coefficient_sigma(equation: dict[str, object], name: str) -> float | None:
    try:
        value = equation["coefficients"][name]["abs_over_std_error"]
        return None if value is None else float(value)
    except (KeyError, TypeError, ValueError):
        return None


def main() -> int:
    args = parse_args()
    if args.derivative_window < 1:
        raise RuntimeError("--derivative-window must be >= 1")
    if args.max_angle_deg <= 0.0 or args.max_angle_deg >= 60.0:
        raise RuntimeError("--max-angle-deg must be > 0 and < 60")
    if args.min_trials < 1:
        raise RuntimeError("--min-trials must be >= 1")
    if args.min_active_samples_per_sign < 1 or args.min_theta_samples_per_sign < 1:
        raise RuntimeError("sample-count gates must be >= 1")
    if args.min_vq_coef_sigma <= 0.0:
        raise RuntimeError("--min-vq-coef-sigma must be > 0")

    with args.input.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("input CSV is empty")
    required = {
        "trial",
        "phase",
        "theta_ref_rad",
        "t_us",
        "theta_rad",
        "theta_rate_rad_s",
        "vel_rad_s",
        "vq_v",
    }
    missing = required - set(rows[0])
    if missing:
        raise RuntimeError("input CSV missing required fields: " + ", ".join(sorted(missing)))

    trial_ids = sorted({int(row["trial"]) for row in rows if row.get("trial")})
    max_angle_rad = math.radians(args.max_angle_deg)
    pooled_x: list[np.ndarray] = []
    pooled_body: list[np.ndarray] = []
    pooled_wheel: list[np.ndarray] = []
    trial_metadata: list[dict[str, object]] = []
    trial_fits: list[dict[str, object]] = []

    for trial in trial_ids:
        trial_rows = [row for row in rows if int(row.get("trial", "0")) == trial]
        x, y_body, y_wheel, meta = extract_trial(
            trial, trial_rows, args.derivative_window, max_angle_rad
        )
        trial_metadata.append(meta)
        if len(x):
            pooled_x.append(x)
            pooled_body.append(y_body)
            pooled_wheel.append(y_wheel)
        fit = trial_fit(trial, x, y_body, y_wheel)
        fit["theta_ref_rad"] = meta.get("theta_ref_rad")
        fit["theta_ref_deg"] = meta.get("theta_ref_deg")
        trial_fits.append(fit)

    if not pooled_x:
        raise RuntimeError("no usable local regression windows found")

    x = np.vstack(pooled_x)
    y_body = np.concatenate(pooled_body)
    y_wheel = np.concatenate(pooled_wheel)
    body = fit_equation(x, y_body)
    wheel = fit_equation(x, y_wheel)

    positive_vq = int(np.sum(x[:, 3] > 1.0e-9))
    negative_vq = int(np.sum(x[:, 3] < -1.0e-9))
    positive_theta = int(np.sum(x[:, 0] > 0.0))
    negative_theta = int(np.sum(x[:, 0] < 0.0))
    reasons: list[str] = []
    if len(trial_ids) < args.min_trials:
        reasons.append(f"trial_count={len(trial_ids)} < min_trials={args.min_trials}")
    if len(x) < len(REGRESSOR_NAMES) + 2:
        reasons.append(f"only {len(x)} usable samples")
    if body["rank"] != len(REGRESSOR_NAMES):
        reasons.append(f"body regression rank={body['rank']}; expected {len(REGRESSOR_NAMES)}")
    if wheel["rank"] != len(REGRESSOR_NAMES):
        reasons.append(f"wheel regression rank={wheel['rank']}; expected {len(REGRESSOR_NAMES)}")
    if positive_vq < args.min_active_samples_per_sign:
        reasons.append(f"positive Vq samples={positive_vq}")
    if negative_vq < args.min_active_samples_per_sign:
        reasons.append(f"negative Vq samples={negative_vq}")
    if positive_theta < args.min_theta_samples_per_sign:
        reasons.append(f"positive theta-error samples={positive_theta}")
    if negative_theta < args.min_theta_samples_per_sign:
        reasons.append(f"negative theta-error samples={negative_theta}")
    for label, equation in (("body", body), ("wheel", wheel)):
        sigma = coefficient_sigma(equation, "vq_v")
        if sigma is None or sigma < args.min_vq_coef_sigma:
            reasons.append(
                f"{label} |Vq coefficient|/SE={sigma}; need >= {args.min_vq_coef_sigma}"
            )

    result = {
        "format": "triwhirl-body-active-local-fit-v1",
        "authority": "CALIBRATION_FIT_ONLY",
        "source": str(args.input),
        "coordinate_contract": (
            "Per trial: theta_error_rad = wrap(theta_rad - theta_ref_rad); "
            "absolute theta_ref_rad is provenance only; no A/B/C classification."
        ),
        "state": ["theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s"],
        "input": "Vq_v",
        "derivative_window": args.derivative_window,
        "max_angle_deg": args.max_angle_deg,
        "trial_count": len(trial_ids),
        "sample_count": int(len(x)),
        "coverage": {
            "positive_vq_samples": positive_vq,
            "negative_vq_samples": negative_vq,
            "positive_theta_error_samples": positive_theta,
            "negative_theta_error_samples": negative_theta,
        },
        "condition_number_normalized": normalized_condition_number(x),
        "body_equation": body,
        "wheel_equation": wheel,
        "trials": trial_metadata,
        "trial_fits": trial_fits,
        "coefficient_variation": coefficient_variation(trial_fits),
        "status": "candidate" if not reasons else "diagnostic_only",
        "reasons": reasons,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    print(
        "local_fit,"
        f"status={result['status']},trials={result['trial_count']},"
        f"samples={result['sample_count']},cond_norm={result['condition_number_normalized']}"
    )
    for reason in reasons:
        print(f"local_fit_reason={reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
