#!/usr/bin/env python3

from __future__ import annotations

import json
import runpy
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PIPELINE = ROOT / "tools/synthesis/hinf/run_pipeline.py"
NAMESPACE = runpy.run_path(str(PIPELINE))
load_validated_calibration = NAMESPACE["load_validated_calibration"]
sha256 = NAMESPACE["sha256"]


def expect_rejected(path: Path, needle: str) -> None:
    try:
        load_validated_calibration(path)
    except RuntimeError as exc:
        assert needle.lower() in str(exc).lower(), str(exc)
    else:
        raise AssertionError(f"expected calibration manifest rejection containing {needle!r}")


def write_manifest(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        linear = root / "linear-model.json"
        linear_payload = {
            "format": "triwhirl-vertex-normalized-linear-model-v1",
            "state": ["theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s"],
            "input": "Vq_v",
        }
        linear.write_text(json.dumps(linear_payload) + "\n", encoding="utf-8")

        manifest = root / "calibration-manifest.json"
        base = {
            "format": "triwhirl-plant-calibration-v1",
            "validation_mode": "external_holdout",
            "validation_sha256": "1" * 64,
            "fit_gate": {"status": "PASS"},
            "parity_acceptance": {"status": "PASS"},
            "synthesis_gate": {"status": "PASS"},
            "vertex_a_deg": 68.0,
            "artifacts": {"linear_model": "linear-model.json"},
            "artifact_sha256": {"linear_model": sha256(linear)},
        }
        write_manifest(manifest, base)

        payload, resolved = load_validated_calibration(manifest)
        assert payload["synthesis_gate"]["status"] == "PASS"
        assert resolved == linear.resolve()

        blocked = dict(base)
        blocked["synthesis_gate"] = {"status": "BLOCKED"}
        write_manifest(manifest, blocked)
        expect_rejected(manifest, "synthesis gate")

        no_holdout = dict(base)
        no_holdout["validation_mode"] = "in_sample_diagnostic"
        write_manifest(manifest, no_holdout)
        expect_rejected(manifest, "holdout")

        parity_unset = dict(base)
        parity_unset["parity_acceptance"] = {"status": "UNSET"}
        write_manifest(manifest, parity_unset)
        expect_rejected(manifest, "parity")

        fit_failed = dict(base)
        fit_failed["fit_gate"] = {"status": "FAIL"}
        write_manifest(manifest, fit_failed)
        expect_rejected(manifest, "fit gate")

        write_manifest(manifest, base)
        linear.write_text(json.dumps({**linear_payload, "tampered": True}) + "\n", encoding="utf-8")
        expect_rejected(manifest, "sha-256 mismatch")

    print("H-infinity validated plant gate: PASS")


if __name__ == "__main__":
    main()
