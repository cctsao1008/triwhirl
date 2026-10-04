#!/usr/bin/env python3
"""Build a vertex-agnostic continuous-time TriWhirl local plant artifact."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert body_active_local_fit.py JSON into a local state-space plant."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--provenance-note", default="")
    return parser.parse_args()


def coefficient(payload: dict[str, object], equation: str, name: str) -> float:
    try:
        return float(payload[equation]["coefficients"][name]["value"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"missing coefficient {equation}.{name}") from exc


def finite_matrix(values: np.ndarray, label: str) -> None:
    if not np.all(np.isfinite(values)):
        raise RuntimeError(f"{label} contains non-finite values")


def main() -> int:
    args = parse_args()
    payload = json.loads(args.input.read_text(encoding="utf-8"))
    if payload.get("format") != "triwhirl-body-active-local-fit-v1":
        raise RuntimeError("unsupported fit format")
    if payload.get("state") != [
        "theta_error_rad",
        "theta_rate_rad_s",
        "wheel_rate_rad_s",
    ]:
        raise RuntimeError("fit uses an unexpected state order")
    if payload.get("input") != "Vq_v":
        raise RuntimeError("fit input must be Vq_v")

    a_theta = coefficient(payload, "body_equation", "theta_error_rad")
    a_rate = coefficient(payload, "body_equation", "theta_rate_rad_s")
    a_wheel = coefficient(payload, "body_equation", "wheel_rate_rad_s")
    b_vq = coefficient(payload, "body_equation", "vq_v")
    body_bias = coefficient(payload, "body_equation", "bias")

    c_theta = coefficient(payload, "wheel_equation", "theta_error_rad")
    c_rate = coefficient(payload, "wheel_equation", "theta_rate_rad_s")
    c_wheel = coefficient(payload, "wheel_equation", "wheel_rate_rad_s")
    d_vq = coefficient(payload, "wheel_equation", "vq_v")
    wheel_bias = coefficient(payload, "wheel_equation", "bias")

    a = np.asarray(
        [
            [0.0, 1.0, 0.0],
            [a_theta, a_rate, a_wheel],
            [c_theta, c_rate, c_wheel],
        ],
        dtype=float,
    )
    bv = np.asarray([[0.0], [b_vq], [d_vq]], dtype=float)
    bias = np.asarray([0.0, body_bias, wheel_bias], dtype=float)
    finite_matrix(a, "A")
    finite_matrix(bv, "Bv")
    finite_matrix(bias, "affine_bias")

    ctrb = np.column_stack((bv, a @ bv, a @ a @ bv))
    eig = np.linalg.eigvals(a)
    trial_provenance = []
    for item in payload.get("trials", []):
        if not isinstance(item, dict):
            continue
        trial_provenance.append(
            {
                "trial": item.get("trial"),
                "theta_ref_rad": item.get("theta_ref_rad"),
                "theta_ref_deg": item.get("theta_ref_deg"),
                "used_samples": item.get("used_samples"),
            }
        )

    nominal = {
        "source_status": payload.get("status", "unknown"),
        "A": a.tolist(),
        "Bv": bv.tolist(),
        "affine_bias": bias.tolist(),
        "open_loop_eigenvalues": [
            {"real": float(value.real), "imag": float(value.imag)} for value in eig
        ],
        "controllability_rank": int(np.linalg.matrix_rank(ctrb)),
        "controllability_condition": float(np.linalg.cond(ctrb)),
        "fit_condition_normalized": payload.get("condition_number_normalized"),
        "body_r_squared": payload.get("body_equation", {}).get("r_squared"),
        "wheel_r_squared": payload.get("wheel_equation", {}).get("r_squared"),
    }

    output = {
        "format": "triwhirl-local-linear-model-v1",
        "source_fit": str(args.input),
        "state": ["theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s"],
        "input": "Vq_v",
        "active_error_mode": "wrapped_theta_ref",
        "coordinate_contract": (
            "Each active-ID trial uses theta_error_rad = wrap(theta_rad - theta_ref_rad). "
            "Absolute theta_ref_rad is provenance only; no A/B/C controller identity exists."
        ),
        "authority": "CALIBRATION_MODEL_ONLY",
        "calibration_trial_count": payload.get("trial_count"),
        "calibration_trials": trial_provenance,
        "coefficient_variation": payload.get("coefficient_variation"),
        "provenance_note": args.provenance_note,
        "nominal": nominal,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    print(
        f"nominal controllability rank={nominal['controllability_rank']} "
        f"cond={nominal['controllability_condition']:.3g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
