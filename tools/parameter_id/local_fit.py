#!/usr/bin/env python3
"""Fit a small-signal TriWhirl local model from telemetry CSV.

The input is the schema-v2 CSV written by tools/parameter_id/acquire.py,
tools/logging/capture.py, or the Web Bluetooth UI. Only fault-free rows with
valid attitude and wheel velocity are used.

Derivatives are estimated by local linear slopes over a configurable telemetry
window instead of differencing adjacent 50 Hz samples. Windows that cross a
Vq transition (or an acquisition phase boundary when phase labels exist) are
rejected. This keeps the fitter from turning encoder/estimator quantization and
step edges into fictitious plant acceleration.
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
    parser.add_argument(
        "--derivative-window", type=int, default=5,
        help="half-window in telemetry samples for local slope derivatives (default: 5)",
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
        if int(row.get("fault_mask", "1"), 0) != 0:
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
                 max_abs_vq: float | None) -> tuple[np.ndarray, list[str]]:
    samples: list[list[float]] = []
    phases: list[str] = []
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
            phases.append(row.get("phase", "") or "")
    if len(samples) < 8:
        raise ValueError("not enough usable fault-free samples (need at least 8)")

    data = np.asarray(samples, dtype=float)
    order = np.argsort(data[:, 0])
    return data[order], [phases[int(i)] for i in order]


def local_slope(time_s: np.ndarray, values: np.ndarray) -> float | None:
    centered_t = time_s - np.mean(time_s)
    denominator = float(centered_t @ centered_t)
    if denominator <= 0.0:
        return None
    centered_y = values - np.mean(values)
    return float(centered_t @ centered_y / denominator)


def build_regression_samples(
    data: np.ndarray,
    phases: list[str],
    half_window: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, int]]:
    if half_window < 1:
        raise ValueError("--derivative-window must be >= 1")
    if len(data) < 2 * half_window + len(REGRESSOR_NAMES) + 2:
        raise ValueError("not enough samples for the requested derivative window")

    regressors: list[list[float]] = []
    theta_ddot: list[float] = []
    wheel_accel: list[float] = []
    rejected_transition = 0
    rejected_time = 0

    for i in range(half_window, len(data) - half_window):
        first = i - half_window
        last = i + half_window + 1
        window = data[first:last]
        times = window[:, 0]
        if np.any(np.diff(times) <= 0.0):
            rejected_time += 1
            continue

        # A derivative window must represent one held input. Otherwise the
        # numerical derivative is dominated by the command edge rather than
        # the plant response to a well-defined Vq.
        vq_window = window[:, 4]
        if float(np.max(vq_window) - np.min(vq_window)) > 1.0e-6:
            rejected_transition += 1
            continue

        center_phase = phases[i]
        if center_phase and any(phases[j] != center_phase for j in range(first, last)):
            rejected_transition += 1
            continue

        theta_slope = local_slope(times, window[:, 2])
        wheel_slope = local_slope(times, window[:, 3])
        if theta_slope is None or wheel_slope is None:
            rejected_time += 1
            continue

        regressors.append([
            data[i, 1],
            data[i, 2],
            data[i, 3],
            data[i, 4],
            1.0,
        ])
        theta_ddot.append(theta_slope)
        wheel_accel.append(wheel_slope)

    if len(regressors) < len(REGRESSOR_NAMES) + 2:
        raise ValueError("not enough held-input samples after derivative filtering")

    return (
        np.asarray(regressors, dtype=float),
        np.asarray(theta_ddot, dtype=float),
        np.asarray(wheel_accel, dtype=float),
        {
            "rejected_input_or_phase_transition": rejected_transition,
            "rejected_nonmonotonic_time": rejected_time,
        },
    )


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

    # Scale only the physical regressors, not the intercept, to distinguish
    # genuine collinearity from harmless unit scaling in the raw matrix.
    physical = x[:, :-1]
    scale = np.std(physical, axis=0)
    scale[scale <= 1.0e-12] = 1.0
    normalized = np.column_stack(((physical - np.mean(physical, axis=0)) / scale,
                                  np.ones(len(x))))
    normalized_singular = np.linalg.svd(normalized, compute_uv=False)
    normalized_condition = None
    if len(normalized_singular) > 0 and normalized_singular[-1] > 0.0:
        normalized_condition = float(normalized_singular[0] / normalized_singular[-1])

    coefficient_map = {
        name: float(value) for name, value in zip(REGRESSOR_NAMES, coefficients)
    }
    stderr_map = None if standard_errors is None else {
        name: float(value) for name, value in zip(REGRESSOR_NAMES, standard_errors)
    }
    vq_signal_to_se = None
    if standard_errors is not None:
        index = REGRESSOR_NAMES.index("vq_v")
        if standard_errors[index] > 0.0:
            vq_signal_to_se = float(abs(coefficients[index]) / standard_errors[index])

    return {
        "coefficients": coefficient_map,
        "coefficient_std_error": stderr_map,
        "vq_coefficient_abs_over_std_error": vq_signal_to_se,
        "rmse": rmse,
        "r2": r2,
        "rank": int(rank),
        "condition_number_raw": condition_number,
        "condition_number_normalized": normalized_condition,
        "singular_values_raw": [float(value) for value in singular],
    }


def main() -> int:
    args = parse_args()
    data, phases = read_samples(args.input, args.max_abs_theta, args.max_abs_vq)
    regressors, theta_ddot, wheel_accel, derivative_stats = build_regression_samples(
        data, phases, args.derivative_window
    )

    result = {
        "format": "triwhirl-local-fit-v2",
        "source": str(args.input),
        "samples_loaded": int(len(data)),
        "samples_fitted": int(len(theta_ddot)),
        "time_span_s": float(data[-1, 0] - data[0, 0]),
        "median_sample_period_s": float(np.median(np.diff(data[:, 0]))),
        "state": list(STATE_NAMES),
        "input": "vq_v",
        "derivative_estimator": {
            "method": "local_linear_slope",
            "half_window_samples": int(args.derivative_window),
            "full_window_samples": int(2 * args.derivative_window + 1),
            **derivative_stats,
        },
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
            "Local regression with held-input derivative windows. Full rank alone does not "
            "make the fit informative; R2, coefficient uncertainty, excitation level, and "
            "physical consistency must be reviewed before using coefficients for control."
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
