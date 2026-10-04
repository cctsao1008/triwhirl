#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.triwhirl_tool.plant_replay import LinearPlant, rk4_step  # noqa: E402


A = (
    (0.0, 1.0, 0.0),
    (3.0, -0.7, 0.15),
    (0.4, -0.1, -1.2),
)
B = (0.0, 2.0, 5.0)
BIAS = (0.0, 0.02, -0.03)
DT_S = 0.005


def write_active_csv(path: Path, theta_refs: list[float], variant: float) -> None:
    initial_states = [
        (0.020 + variant, 0.050, 0.000),
        (-0.020 - variant, -0.040, 0.200),
        (0.015 + 0.5 * variant, -0.030, -0.150),
        (-0.015 - 0.5 * variant, 0.020, 0.100),
    ]
    signs = (1.0, -1.0, -1.0, 1.0)
    plant = LinearPlant(name="synthetic", A=A, B=B, bias=BIAS)
    fieldnames = [
        "schema_version",
        "trial",
        "phase",
        "theta_ref_rad",
        "t_us",
        "theta_rad",
        "theta_rate_rad_s",
        "vel_rad_s",
        "vq_v",
        "fault_mask",
        "attitude_ok",
        "vel_valid",
    ]

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for trial, (theta_ref, initial, sign) in enumerate(
            zip(theta_refs, initial_states, signs), start=1
        ):
            state = initial
            for index in range(120):
                active = index < 60
                vq = 0.08 * sign if active else 0.0
                phase = "active" if active else "zero_vector"
                writer.writerow(
                    {
                        "schema_version": "2",
                        "trial": str(trial),
                        "phase": phase,
                        "theta_ref_rad": f"{theta_ref:.12f}",
                        "t_us": str(int(round(index * DT_S * 1.0e6))),
                        "theta_rad": f"{theta_ref + state[0]:.12f}",
                        "theta_rate_rad_s": f"{state[1]:.12f}",
                        "vel_rad_s": f"{state[2]:.12f}",
                        "vq_v": f"{vq:.12f}",
                        "fault_mask": "0",
                        "attitude_ok": "1",
                        "vel_valid": "1",
                    }
                )
                state = rk4_step(plant, state, vq, DT_S, include_bias=True)


def run(command: list[str]) -> None:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if completed.returncode != 0:
        raise AssertionError(
            f"command failed rc={completed.returncode}: {' '.join(command)}\n"
            f"{completed.stdout}"
        )


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        calibration = root / "calibration.csv"
        validation = root / "validation.csv"
        output_dir = root / "out"

        calibration_refs = [0.40, 1.70, -2.20, 2.80]
        validation_refs = [-0.70, 2.00, -2.80, 1.10]
        write_active_csv(calibration, calibration_refs, 0.0)
        write_active_csv(validation, validation_refs, 0.002)

        run(
            [
                sys.executable,
                "tools/twtool.py",
                "plant",
                "calibrate",
                str(calibration),
                "--validation",
                str(validation),
                "--derivative-window",
                "2",
                "--max-angle-deg",
                "8",
                "--max-theta-rollout-rmse-deg",
                "0.01",
                "--max-rate-rollout-rmse",
                "0.001",
                "--max-wheel-rollout-rmse",
                "0.001",
                "--output-dir",
                str(output_dir),
                "--provenance-note",
                "synthetic contract test only; no hardware authority",
            ]
        )

        fit = json.loads((output_dir / "active-local-fit.json").read_text(encoding="utf-8"))
        assert fit["format"] == "triwhirl-body-active-local-fit-v1"
        assert fit["authority"] == "CALIBRATION_FIT_ONLY"
        assert fit["status"] == "candidate"
        assert fit["trial_count"] == 4
        assert "vertex_fits" not in fit
        refs = [float(item["theta_ref_rad"]) for item in fit["trials"]]
        for measured, expected in zip(refs, calibration_refs):
            assert abs(measured - expected) < 1.0e-9

        model = json.loads((output_dir / "linear-model.json").read_text(encoding="utf-8"))
        assert model["format"] == "triwhirl-local-linear-model-v1"
        assert model["active_error_mode"] == "wrapped_theta_ref"
        assert model["authority"] == "CALIBRATION_MODEL_ONLY"
        assert "plants" not in model
        assert model["nominal"]["source_status"] == "candidate"
        assert model["nominal"]["controllability_rank"] == 3
        assert model["calibration_trial_count"] == 4

        parity = json.loads((output_dir / "replay-parity.json").read_text(encoding="utf-8"))
        assert parity["model_format"] == "triwhirl-local-linear-model-v1"
        assert parity["data_schema"] == "active_id_local"
        assert parity["active_error_mode"] == "wrapped_theta_ref"
        assert parity["validation_mode"] == "external_holdout"
        assert parity["acceptance"]["status"] == "PASS"
        assert parity["promotion_gate"]["status"] == "PASS"
        assert parity["segment_count"] == 4
        assert parity["metrics"]["theta_error"]["rollout_rmse_deg"] < 0.01

        manifest = json.loads(
            (output_dir / "calibration-manifest.json").read_text(encoding="utf-8")
        )
        assert manifest["format"] == "triwhirl-plant-calibration-v2"
        assert manifest["authority"] == "LOCAL_MODEL_EVIDENCE_ONLY"
        assert manifest["model_format"] == "triwhirl-local-linear-model-v1"
        assert manifest["validation_mode"] == "external_holdout"
        assert manifest["promotion_gate"]["status"] == "PASS"
        assert manifest["legacy_hinf_compatible"] is False
        assert "vertices" not in manifest
        assert "vertex_a_deg" not in manifest

        # The default replay command must also continue to dispatch historical
        # A/B/C artifacts to the legacy replay implementation.
        legacy_model = root / "legacy-model.json"
        legacy_summary = root / "legacy-replay.json"
        legacy_model.write_text(
            json.dumps(
                {
                    "format": "triwhirl-vertex-normalized-linear-model-v1",
                    "state": [
                        "theta_error_rad",
                        "theta_rate_rad_s",
                        "wheel_rate_rad_s",
                    ],
                    "input": "Vq_v",
                    "plants": [
                        {
                            "vertex_id": "A",
                            "nominal_center_deg": math.degrees(calibration_refs[0]),
                            "A": [list(row) for row in A],
                            "Bv": [[value] for value in B],
                            "affine_bias": list(BIAS),
                        }
                    ],
                    "nominal": {
                        "A": [list(row) for row in A],
                        "Bv": [[value] for value in B],
                        "affine_bias": list(BIAS),
                    },
                }
            ),
            encoding="utf-8",
        )
        run(
            [
                sys.executable,
                "tools/twtool.py",
                "plant",
                "replay",
                str(legacy_model),
                str(calibration),
                "--plant",
                "nominal",
                "--max-angle-deg",
                "8",
                "-o",
                str(legacy_summary),
            ]
        )
        legacy = json.loads(legacy_summary.read_text(encoding="utf-8"))
        assert legacy["format"] == "triwhirl-replay-parity-v1"
        assert legacy["data_schema"] == "active_id"


if __name__ == "__main__":
    main()
