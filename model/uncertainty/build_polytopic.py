#!/usr/bin/env python3
"""Build a compact empirical polytopic plant set from vertex-normalized models.

The selected physical upright vertices are treated as measured samples of one
shared local plant family.  The convex hull is intentionally kept small: one
plant vertex per selected identification location, plus a separate nominal
matrix for reporting/synthesis initialization.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a measured-vertex polytopic uncertainty artifact."
    )
    parser.add_argument("input", type=Path, help="linear model JSON")
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def elementwise_envelope(values: np.ndarray, nominal: np.ndarray) -> dict[str, object]:
    delta = values - nominal
    abs_max = np.max(np.abs(delta), axis=0)
    denom = np.maximum(np.abs(nominal), 1.0e-9)
    rel_max = abs_max / denom
    return {
        "absolute_max_delta": abs_max.tolist(),
        "relative_max_delta": rel_max.tolist(),
        "min": np.min(values, axis=0).tolist(),
        "max": np.max(values, axis=0).tolist(),
    }


def main() -> int:
    args = parse_args()
    linear = json.loads(args.input.read_text(encoding="utf-8"))
    plants = linear.get("plants")
    nominal = linear.get("nominal")
    if not isinstance(plants, list) or not plants:
        raise RuntimeError("linear model contains no plant vertices")
    if not isinstance(nominal, dict):
        raise RuntimeError("linear model contains no nominal plant")

    a_values = np.asarray([plant["A"] for plant in plants], dtype=float)
    b_values = np.asarray([plant["Bv"] for plant in plants], dtype=float)
    a_nom = np.asarray(nominal["A"], dtype=float)
    b_nom = np.asarray(nominal["Bv"], dtype=float)

    poly_vertices: list[dict[str, object]] = []
    for plant in plants:
        poly_vertices.append(
            {
                "id": f"measured_{plant['vertex_id']}",
                "source_vertex": plant["vertex_id"],
                "A": plant["A"],
                "Bv": plant["Bv"],
                "source_status": plant.get("source_status", "unknown"),
                "sample_count": plant.get("sample_count", 0),
                "fit_condition_normalized": plant.get("fit_condition_normalized"),
            }
        )

    output = {
        "format": "triwhirl-polytopic-uncertainty-v1",
        "source_linear_model": str(args.input),
        "state": linear.get("state"),
        "input": linear.get("input"),
        "selected_vertices": linear.get("selected_vertices"),
        "nominal": {"A": nominal["A"], "Bv": nominal["Bv"]},
        "polytope": {
            "definition": "convex hull of selected measured local vertex models",
            "vertex_count": len(poly_vertices),
            "vertices": poly_vertices,
        },
        "empirical_envelope": {
            "A": elementwise_envelope(a_values, a_nom),
            "Bv": elementwise_envelope(b_values, b_nom),
        },
        "interpretation": (
            "The physical robot may balance on A, B, or C with one vertex-normalized controller. "
            "Identification data may come from any subset of those locations.  This artifact uses "
            "the selected measured locations as uncertainty samples; absence of a location from the "
            "fit does not mean the controller is restricted to the measured locations."
        ),
        "limitations": [
            "A convex hull of measured local fits is an empirical uncertainty model, not proof that all unmeasured conditions lie inside it.",
            "Large coefficient differences should be traced to excitation/conditioning or represented conservatively rather than averaged away.",
            "Timing, supply effectiveness, wheel-speed dependence, and residual equilibrium bias may require additional uncertainty channels.",
        ],
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    print(f"polytopic vertices: {len(poly_vertices)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
