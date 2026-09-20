#!/usr/bin/env python3
"""Build vertex-normalized continuous-time state-space plants from active-fit JSON.

The controller state is always expressed in the shared 120-degree-periodic
upright coordinate:
    x = [theta_error, theta_rate, wheel_rate]^T
    u = Vq

Identification may use one, two, or three physical upright vertices.  The
resulting per-vertex fits are samples of one shared local plant family; they do
not imply separate controllers for A/B/C.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert per-vertex active-fit JSON into local state-space plants."
    )
    parser.add_argument("input", type=Path, help="body_active_fit.py JSON output")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument(
        "--vertices",
        default="auto",
        help="comma-separated vertex IDs (A,B,C) or 'auto' for candidate vertices",
    )
    parser.add_argument(
        "--nominal",
        default="mean",
        help="nominal plant: 'mean' or one selected vertex ID",
    )
    parser.add_argument(
        "--provenance-note",
        default="",
        help="free-form note carried into the generated artifact",
    )
    return parser.parse_args()


def parse_vertices(spec: str, payload: dict[str, object]) -> list[str]:
    if spec.strip().lower() == "auto":
        vertices = [str(v) for v in payload.get("candidate_vertices", [])]
        if not vertices:
            raise RuntimeError(
                "no candidate vertices in fit JSON; pass --vertices explicitly to use diagnostic fits"
            )
        return vertices
    vertices: list[str] = []
    for token in spec.split(","):
        vertex = token.strip().upper()
        if not vertex:
            continue
        if vertex not in {"A", "B", "C"}:
            raise RuntimeError(f"invalid vertex '{vertex}'; expected A,B,C")
        if vertex not in vertices:
            vertices.append(vertex)
    if not vertices:
        raise RuntimeError("--vertices selected no vertices")
    return vertices


def coefficient(fit: dict[str, object], equation: str, name: str) -> float:
    try:
        return float(fit[equation]["coefficients"][name]["value"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"missing coefficient {equation}.{name}") from exc


def build_plant(vertex: str, fit: dict[str, object]) -> dict[str, object]:
    if "body_equation" not in fit or "wheel_equation" not in fit:
        raise RuntimeError(f"vertex {vertex} has no fitted equations")

    a_theta = coefficient(fit, "body_equation", "theta_error_gyro_rad")
    a_rate = coefficient(fit, "body_equation", "theta_rate_rad_s")
    a_wheel = coefficient(fit, "body_equation", "wheel_rate_rad_s")
    b_vq = coefficient(fit, "body_equation", "vq_v")
    body_bias = coefficient(fit, "body_equation", "bias")

    c_theta = coefficient(fit, "wheel_equation", "theta_error_gyro_rad")
    c_rate = coefficient(fit, "wheel_equation", "theta_rate_rad_s")
    c_wheel = coefficient(fit, "wheel_equation", "wheel_rate_rad_s")
    d_vq = coefficient(fit, "wheel_equation", "vq_v")
    wheel_bias = coefficient(fit, "wheel_equation", "bias")

    a = np.asarray(
        [
            [0.0, 1.0, 0.0],
            [a_theta, a_rate, a_wheel],
            [c_theta, c_rate, c_wheel],
        ],
        dtype=float,
    )
    bv = np.asarray([[0.0], [b_vq], [d_vq]], dtype=float)
    ctrb = np.column_stack((bv, a @ bv, a @ a @ bv))
    eig = np.linalg.eigvals(a)

    return {
        "vertex_id": vertex,
        "source_status": fit.get("status", "unknown"),
        "sample_count": int(fit.get("sample_count", 0)),
        "nominal_center_deg": float(fit.get("nominal_center_deg", 0.0)),
        "A": a.tolist(),
        "Bv": bv.tolist(),
        "affine_bias": [0.0, body_bias, wheel_bias],
        "open_loop_eigenvalues": [
            {"real": float(value.real), "imag": float(value.imag)} for value in eig
        ],
        "controllability_rank": int(np.linalg.matrix_rank(ctrb)),
        "controllability_condition": float(np.linalg.cond(ctrb)),
        "fit_condition_normalized": fit.get("condition_number_normalized"),
        "body_r_squared": fit["body_equation"].get("r_squared"),
        "wheel_r_squared": fit["wheel_equation"].get("r_squared"),
    }


def mean_plant(plants: list[dict[str, object]]) -> dict[str, object]:
    a_stack = np.asarray([plant["A"] for plant in plants], dtype=float)
    b_stack = np.asarray([plant["Bv"] for plant in plants], dtype=float)
    bias_stack = np.asarray([plant["affine_bias"] for plant in plants], dtype=float)
    a = np.mean(a_stack, axis=0)
    bv = np.mean(b_stack, axis=0)
    bias = np.mean(bias_stack, axis=0)
    ctrb = np.column_stack((bv, a @ bv, a @ a @ bv))
    eig = np.linalg.eigvals(a)
    return {
        "kind": "arithmetic_mean_of_selected_vertex_models",
        "A": a.tolist(),
        "Bv": bv.tolist(),
        "affine_bias": bias.tolist(),
        "open_loop_eigenvalues": [
            {"real": float(value.real), "imag": float(value.imag)} for value in eig
        ],
        "controllability_rank": int(np.linalg.matrix_rank(ctrb)),
        "controllability_condition": float(np.linalg.cond(ctrb)),
    }


def main() -> int:
    args = parse_args()
    payload = json.loads(args.input.read_text(encoding="utf-8"))
    vertex_fits = payload.get("vertex_fits")
    if not isinstance(vertex_fits, dict):
        raise RuntimeError("input JSON has no vertex_fits map")

    vertices = parse_vertices(args.vertices, payload)
    plants: list[dict[str, object]] = []
    for vertex in vertices:
        fit = vertex_fits.get(vertex)
        if not isinstance(fit, dict):
            raise RuntimeError(f"input JSON has no fit for vertex {vertex}")
        plants.append(build_plant(vertex, fit))

    nominal_spec = args.nominal.strip().upper()
    if args.nominal.strip().lower() == "mean":
        nominal = mean_plant(plants)
    else:
        if nominal_spec not in vertices:
            raise RuntimeError("--nominal vertex must be included in --vertices")
        selected = next(plant for plant in plants if plant["vertex_id"] == nominal_spec)
        nominal = {
            "kind": f"selected_vertex_{nominal_spec}",
            "A": selected["A"],
            "Bv": selected["Bv"],
            "affine_bias": selected["affine_bias"],
            "open_loop_eigenvalues": selected["open_loop_eigenvalues"],
            "controllability_rank": selected["controllability_rank"],
            "controllability_condition": selected["controllability_condition"],
        }

    output = {
        "format": "triwhirl-vertex-normalized-linear-model-v1",
        "source_fit": str(args.input),
        "state": ["theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s"],
        "input": "Vq_v",
        "selected_vertices": vertices,
        "coordinate_contract": (
            "At runtime compute theta_error with 120-degree periodic wrapping around the "
            "upright reference and apply one shared controller. A/B/C labels are optional "
            "for control and remain useful for logging and identification provenance."
        ),
        "identification_contract": (
            "One, two, or three physical vertices may be used as plant samples.  Vertex labels "
            "describe identification locations, not separate controller identities."
        ),
        "affine_bias_note": (
            "Affine fit bias is reported for diagnostics.  Robust-control synthesis should use "
            "equilibrium-centered local coordinates and treat residual bias as disturbance/model error."
        ),
        "provenance_note": args.provenance_note,
        "plants": plants,
        "nominal": nominal,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    print("selected vertices: " + ", ".join(vertices))
    print(
        f"nominal controllability rank={nominal['controllability_rank']} "
        f"cond={nominal['controllability_condition']:.3g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
