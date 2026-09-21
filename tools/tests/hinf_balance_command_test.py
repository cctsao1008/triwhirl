#!/usr/bin/env python3

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.synthesis.hinf.balance_command import load_command_values, render_balance_command


def write_payload(root: Path, payload: dict) -> Path:
    path = root / "controller.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def main() -> None:
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        payload = {
            "format": "triwhirl-hinf-state-feedback-v1",
            "all_vertices_stable": True,
            "K_inf": [12.5, 1.25, -0.08],
            "performance_scales": {
                "Vq_v": 1.2,
                "wheel_rate_rad_s": 40.0,
            },
        }
        source = write_payload(root, payload)
        values = load_command_values(
            source,
            theta_reference_deg=68.0,
            capture_deg=6.0,
            fall_deg=24.0,
            vq_limit_v=None,
            wheel_limit_rad_s=None,
        )
        assert values == (12.5, 1.25, -0.08, 68.0, 6.0, 24.0, 1.2, 40.0)
        assert render_balance_command(values) == (
            "balance config 12.5 1.25 -0.08 68 6 24 1.2 40"
        )

        payload["all_vertices_stable"] = False
        source = write_payload(root, payload)
        try:
            load_command_values(
                source,
                theta_reference_deg=68.0,
                capture_deg=6.0,
                fall_deg=24.0,
                vq_limit_v=None,
                wheel_limit_rad_s=None,
            )
        except ValueError as exc:
            assert "not all plant vertices are stable" in str(exc)
        else:
            raise AssertionError("unstable synthesis must not generate a command")

        payload["all_vertices_stable"] = True
        source = write_payload(root, payload)
        try:
            load_command_values(
                source,
                theta_reference_deg=68.0,
                capture_deg=24.0,
                fall_deg=24.0,
                vq_limit_v=None,
                wheel_limit_rad_s=None,
            )
        except ValueError as exc:
            assert "capture angle" in str(exc)
        else:
            raise AssertionError("invalid capture/fall envelope must be rejected")


if __name__ == "__main__":
    main()
