#!/usr/bin/env python3
"""Fit a small-signal TriWhirl local model from telemetry CSV.

The input is the schema-v2 CSV written by tools/logging/capture.py or the
Web Bluetooth UI. Only fault-free rows with valid attitude and wheel velocity
are used. This tool estimates continuous-time local equations with Vq as the
physical actuator input; it does not claim torque measurement.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

STATE_NAMES = ("theta_rad", "theta_rate_rad_s", "vel_rad_s")
REGRESSOR_NAMES = (*STATE_NAMES, "vq_v", "bias")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit a local continuous-time TriWhirl model from telemetry CSV."
    )
    parser.add_argument("input", type=Path, help="schema-v2 telemetry CSV")
    parser.add_argument(
        "-o", "--output", type=Path, default=None,
        help="optional JSON result path; default prints JSON to stdout",
    )
    parser.add_argument(
        "--max-abs-theta", type=float, default=None,
        help="optional local-region filter |theta| <= value [rad]",
    )
    parser.add_argument(
        "--max-abs-vq", type=float, default=None,
        help="optional actuator filter |Vq| <= value [V]",
    )
    return parser.parse_args()


def as_float(row: dict[str, str], name: str) -> float:
    try:
        return float(row[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid or missing column {name!r}") from exc


def usable(row: dict[str, str], max_abs_theta: float | None,
           max_abs_vq: float | None) -> bool:
    try:
        if int(row.get("schema_version", "0")) != 2:
            return False
        if int(row.get("fault_mask", "1")) != 0:
            return False
        if int(row.get("attitude_ok", "0")) != 1:
            return False
        if int(row.get("vel_valid", "0")) != 1:
            return False
        theta = as_float(row, "theta_rad")
        vq = as_float(row, "vq_v")
        values = [
            as_float(row, "t_us"), theta,
            as_float(row, "theta_rate_rad_s"),
            as_float(row, "vel_rad_s"), vq,
        ]
        if not all(math.isfinite(value) for value in values):
            return False
        if max_abs_theta is not None and abs(theta) > max_abs_theta:
            return False
        if max_abs_vq is not None and abs(vq) > max_abs_vq:
            return False
        return True
    except (ValueError, TypeError):
        return False


def read_samples(path: Path, max_abs_theta: float | None,
                 max_abs_vq: float | None) -> np.ndarray:
    samples: list[list[float]] = []
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            if not usable(row, max_abs_theta, max_abs_vq):
                continue
            samples.append([
                as_float(row, "t_us") * 1.0e-6,
                as_float(row, "theta_rad"),
                as_float(row, "theta_rate_rad_s"),
                as_float(row, "vel_rad_s"),
                as_float(row, "vq_v"),
            ])
    if len(samples) < 8:
        raise ValueError("not enough usable fault-free samples (need at least 8)")
    data = np.asarray(samples, dtype=float)
    order = np.argsort(data[:, 0])
    return data[order]


def central_derivative(time_s: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    dt = time_s[2:] - time_s[:-2]
    good = dt > 0.0
    derivative = np.zeros_like(dt)
    derivative[good] = (values[2:][good] - values[:-2][good]) / dt[good]
    return derivative, good


def fit_equation(x: np.ndarray, y: np.ndarray) -> dict[str, object]:
    coefficients, _, rank, singular = np.linalg.lstsq(x, y, rcond=None)
    predicted = x @ coefficients
    residual = y - predicted
    ss_res = float(residual @ residual)
    centered = y - np.mean(y)
    ss_tot = float(centered @ centered)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else None
    rmse = math.sqrt(ss_res / len(y))

    degrees_of_freedom = len(y) - int(rank)
    standard_errors: np.ndarray | None = None
    if degrees_of_freedom > 0:
        residual_variance = ss_res / degrees_of_freedom
        covariance = residual_variance * np.linalg.pinv(x.T @ x)
        diagonal = np.maximum(np.diag(covariance), 0.0)
        standard_errors = np.sqrt(diagonal)

    condition_number = None
    if len(singular) > 0 and singular[-1] > 0.0:
        condition_number = float(singular[0] / singular[-1])

    return {
        "coefficients": {
            name: float(value) for name, value in zip(REGRESSOR_NAMES, coefficients)
        },
        "coefficient_std_error": None if standard_errors is None else {
            name: float(value) for name, value in zip(REGRESSOR_NAMES, standard_errors)
        },
        "rmse": rmse,
        "r2": r2,
        "rank": int(rank),
        "condition_number": condition_number,
        "singular_values": [float(value) for value in singular],
    }


def main() -> int:
    args = parse_args()
    data = read_samples(args.input, args.max_abs_theta, args.max_abs_vq)

    time_s = data[:, 0]
    theta = data[:, 1]
    theta_rate = data[:, 2]
    wheel_rate = data[:, 3]
    vq = data[:, 4]

    theta_ddot, good_theta = central_derivative(time_s, theta_rate)
    wheel_accel, good_wheel = central_derivative(time_s, wheel_rate)
    good = good_theta & good_wheel

    center = slice(1, -1)
    regressors = np.column_stack((
        theta[center],
        theta_rate[center],
        wheel_rate[center],
        vq[center],
        np.ones(len(theta) - 2),
    ))
    regressors = regressors[good]
    theta_ddot = theta_ddot[good]
    wheel_accel = wheel_accel[good]

    if len(theta_ddot) < len(REGRESSOR_NAMES) + 2:
        raise ValueError("not enough time-contiguous samples after derivative filtering")

    result = {
        "format": "triwhirl-local-fit-v1",
        "source": str(args.input),
        "samples_loaded": int(len(data)),
        "samples_fitted": int(len(theta_ddot)),
        "time_span_s": float(time_s[-1] - time_s[0]),
        "median_sample_period_s": float(np.median(np.diff(time_s))),
        "state": list(STATE_NAMES),
        "input": "vq_v",
        "equations": {
            "theta_ddot_rad_s2": fit_equation(regressors, theta_ddot),
            "wheel_accel_rad_s2": fit_equation(regressors, wheel_accel),
        },
        "filters": {
            "fault_mask": 0,
            "attitude_ok": 1,
            "vel_valid": 1,
            "max_abs_theta_rad": args.max_abs_theta,
            "max_abs_vq_v": args.max_abs_vq,
        },
        "interpretation": (
            "Preliminary local regression only. Coefficients become plant evidence "
            "only when the source log comes from a controlled near-upright experiment."
        ),
    }

    text = json.dumps(result, indent=2, sort_keys=False) + "\n"
    if args.output is None:
        print(text, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
