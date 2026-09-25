#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.triwhirl_tool.plant_replay import (  # noqa: E402
    LinearPlant,
    replay,
    rk4_step,
    write_replay_csv,
)


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        model_path = root / "model.json"
        data_path = root / "active.csv"
        output_path = root / "replay.csv"

        a = [
            [0.0, 1.0, 0.0],
            [2.0, -0.3, 0.1],
            [0.2, 0.0, -0.4],
        ]
        b = [[0.0], [1.5], [4.0]]
        bias = [0.0, 0.01, -0.02]
        model = {
            "format": "triwhirl-vertex-normalized-linear-model-v1",
            "state": ["theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s"],
            "input": "Vq_v",
            "plants": [
                {
                    "vertex_id": "A",
                    "nominal_center_deg": 68.0,
                    "A": a,
                    "Bv": b,
                    "affine_bias": bias,
                }
            ],
            "nominal": {
                "A": a,
                "Bv": b,
                "affine_bias": bias,
            },
        }
        model_path.write_text(json.dumps(model), encoding="utf-8")

        plant = LinearPlant(
            name="vertex_A",
            A=tuple(tuple(float(v) for v in row) for row in a),
            B=(0.0, 1.5, 4.0),
            bias=(0.0, 0.01, -0.02),
            vertex_id="A",
            nominal_center_deg=68.0,
        )
        state = (math.radians(2.0), 0.1, -0.2)
        dt = 0.01
        theta_ref = math.radians(68.0)
        rows: list[dict[str, str]] = []
        t_s = 0.0
        for index in range(80):
            u_v = 0.20 if index < 40 else -0.15
            rows.append(
                {
                    "trial": "1",
                    "t_us": str(int(round(t_s * 1.0e6))),
                    "phase": "active",
                    "theta_rad": f"{theta_ref + state[0]:.12f}",
                    "theta_ref_rad": f"{theta_ref:.12f}",
                    "theta_rate_rad_s": f"{state[1]:.12f}",
                    "vel_rad_s": f"{state[2]:.12f}",
                    "vq_v": f"{u_v:.12f}",
                    "vertex_id": "A",
                }
            )
            state = rk4_step(plant, state, u_v, dt, include_bias=True)
            t_s += dt

        with data_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        summary, samples = replay(
            model_path,
            data_path,
            plant_selection="auto",
            max_angle_deg=20.0,
            include_bias=True,
        )
        assert summary["data_schema"] == "active_id"
        assert summary["segment_count"] == 1
        assert summary["interval_count"] == len(samples)
        assert summary["segments"][0]["plant"] == "vertex_A"

        metrics = summary["metrics"]
        assert metrics["theta_error"]["one_step_rmse_deg"] < 1.0e-3
        assert metrics["theta_rate"]["one_step_rmse"] < 1.0e-3
        assert metrics["wheel_rate"]["one_step_rmse"] < 1.0e-3
        assert metrics["theta_error"]["rollout_rmse_deg"] < 0.01
        assert metrics["theta_rate"]["rollout_rmse"] < 1.0e-6
        assert metrics["wheel_rate"]["rollout_rmse"] < 1.0e-6

        write_replay_csv(output_path, samples)
        assert output_path.stat().st_size > 100


if __name__ == "__main__":
    main()
