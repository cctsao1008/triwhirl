#!/usr/bin/env python3
"""Build robust-control artifacts only from a validated plant calibration manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build the empirical uncertainty model and synthesize one H-infinity gain "
            "from a plant calibration manifest whose independent replay gate is PASS."
        )
    )
    parser.add_argument(
        "input",
        type=Path,
        help="calibration-manifest.json from 'twtool plant calibrate'",
    )
    parser.add_argument("--capture-deg", type=float, default=6.0)
    parser.add_argument("--fall-deg", type=float, default=24.0)
    parser.add_argument("--vq-limit-v", type=float)
    parser.add_argument("--wheel-limit-rad-s", type=float)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/hinf"))
    parser.add_argument("--solver", default="CLARABEL")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_manifest_artifact(manifest_path: Path, raw_path: str) -> Path:
    candidate = Path(raw_path)
    if candidate.is_absolute():
        return candidate

    local = manifest_path.parent / candidate
    if local.exists():
        return local
    if candidate.exists():
        return candidate

    # Older manifests may have stored a repo-relative path that already includes
    # the output directory.  The basename still identifies the colocated artifact.
    colocated = manifest_path.parent / candidate.name
    if colocated.exists():
        return colocated
    return local


def load_validated_calibration(manifest_path: Path) -> tuple[dict[str, object], Path]:
    manifest_path = manifest_path.resolve()
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if payload.get("format") != "triwhirl-plant-calibration-v1":
        raise RuntimeError(
            "H-infinity synthesis now requires a triwhirl-plant-calibration-v1 manifest; "
            "run 'python tools/twtool.py plant calibrate ...' first"
        )

    validation_mode = payload.get("validation_mode")
    if validation_mode != "external_holdout":
        raise RuntimeError(
            f"calibration validation_mode={validation_mode!r}; independent holdout validation is required"
        )
    if not payload.get("validation_sha256"):
        raise RuntimeError("calibration manifest has no validation dataset SHA-256 provenance")

    fit_gate = payload.get("fit_gate")
    if not isinstance(fit_gate, dict) or fit_gate.get("status") != "PASS":
        status = fit_gate.get("status") if isinstance(fit_gate, dict) else "missing"
        raise RuntimeError(f"plant fit gate is {status}; synthesis requires PASS")

    parity = payload.get("parity_acceptance")
    if not isinstance(parity, dict) or parity.get("status") != "PASS":
        status = parity.get("status") if isinstance(parity, dict) else "missing"
        raise RuntimeError(
            f"holdout replay parity acceptance is {status}; explicit thresholds must PASS"
        )

    synthesis_gate = payload.get("synthesis_gate")
    if not isinstance(synthesis_gate, dict) or synthesis_gate.get("status") != "PASS":
        status = synthesis_gate.get("status") if isinstance(synthesis_gate, dict) else "missing"
        raise RuntimeError(f"plant calibration synthesis gate is {status}; expected PASS")

    artifacts = payload.get("artifacts")
    hashes = payload.get("artifact_sha256")
    if not isinstance(artifacts, dict) or not isinstance(hashes, dict):
        raise RuntimeError("calibration manifest lacks artifact paths or artifact SHA-256 hashes")
    linear_raw = artifacts.get("linear_model")
    expected_hash = hashes.get("linear_model")
    if not isinstance(linear_raw, str) or not linear_raw:
        raise RuntimeError("calibration manifest has no linear_model artifact path")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise RuntimeError("calibration manifest has no valid linear_model SHA-256")

    linear_path = _resolve_manifest_artifact(manifest_path, linear_raw).resolve()
    if not linear_path.exists():
        raise RuntimeError(f"validated linear model not found: {linear_path}")
    actual_hash = sha256(linear_path)
    if actual_hash.lower() != expected_hash.lower():
        raise RuntimeError(
            "validated linear model SHA-256 mismatch; calibration artifact changed after replay validation"
        )

    linear = json.loads(linear_path.read_text(encoding="utf-8"))
    if linear.get("format") != "triwhirl-vertex-normalized-linear-model-v1":
        raise RuntimeError("validated linear model has an unsupported format")
    if linear.get("state") != [
        "theta_error_rad",
        "theta_rate_rad_s",
        "wheel_rate_rad_s",
    ]:
        raise RuntimeError("validated linear model state order does not match the controller contract")
    if linear.get("input") != "Vq_v":
        raise RuntimeError("validated linear model input is not Vq_v")

    return payload, linear_path


def run(command: list[str]) -> None:
    print("+ " + " ".join(command))
    subprocess.run(command, check=True)


def main() -> int:
    args = parse_args()
    repo = Path(__file__).resolve().parents[3]

    try:
        manifest, linear = load_validated_calibration(args.input)
    except (OSError, RuntimeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"error: {exc}")
        return 2

    vertex_a_deg = float(manifest.get("vertex_a_deg", 68.0))
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    poly = out / "validated-polytope.json"
    controller = out / "validated-hinf.json"
    balance_command = out / "validated-balance-command.txt"
    provenance = out / "synthesis-provenance.json"

    try:
        run(
            [
                sys.executable,
                str(repo / "model/uncertainty/build_polytopic.py"),
                str(linear),
                "-o",
                str(poly),
            ]
        )
        run(
            [
                sys.executable,
                str(repo / "tools/synthesis/hinf/synthesize.py"),
                str(poly),
                "--solver",
                args.solver,
                "-o",
                str(controller),
            ]
        )

        command = [
            sys.executable,
            str(repo / "tools/synthesis/hinf/balance_command.py"),
            str(controller),
            "--theta-reference-deg",
            str(vertex_a_deg),
            "--capture-deg",
            str(args.capture_deg),
            "--fall-deg",
            str(args.fall_deg),
        ]
        if args.vq_limit_v is not None:
            command.extend(("--vq-limit-v", str(args.vq_limit_v)))
        if args.wheel_limit_rad_s is not None:
            command.extend(("--wheel-limit-rad-s", str(args.wheel_limit_rad_s)))
        print("+ " + " ".join(command) + f" > {balance_command}")
        with balance_command.open("w", encoding="utf-8") as stream:
            subprocess.run(command, check=True, stdout=stream, text=True)
    except subprocess.CalledProcessError as exc:
        print(f"error: synthesis pipeline command failed with exit code {exc.returncode}")
        return 1

    provenance_payload = {
        "format": "triwhirl-hinf-synthesis-provenance-v1",
        "calibration_manifest": str(args.input),
        "calibration_manifest_sha256": sha256(args.input),
        "validated_linear_model": str(linear),
        "validated_linear_model_sha256": sha256(linear),
        "validation_mode": manifest.get("validation_mode"),
        "validation_sha256": manifest.get("validation_sha256"),
        "fit_gate": manifest.get("fit_gate"),
        "parity_acceptance": manifest.get("parity_acceptance"),
        "synthesis_gate": manifest.get("synthesis_gate"),
        "vertex_a_deg": vertex_a_deg,
        "solver": args.solver,
        "deployment_limits": {
            "capture_deg": args.capture_deg,
            "fall_deg": args.fall_deg,
            "vq_limit_v": args.vq_limit_v,
            "wheel_limit_rad_s": args.wheel_limit_rad_s,
        },
        "outputs": {
            "polytope": str(poly),
            "polytope_sha256": sha256(poly),
            "controller": str(controller),
            "controller_sha256": sha256(controller),
            "balance_command": str(balance_command),
            "balance_command_sha256": sha256(balance_command),
        },
    }
    provenance.write_text(json.dumps(provenance_payload, indent=2) + "\n", encoding="utf-8")

    print("pipeline complete")
    print(f"validated manifest: {args.input}")
    print(f"validated linear:   {linear}")
    print(f"polytope:           {poly}")
    print(f"controller:         {controller}")
    print(f"balance command:    {balance_command}")
    print(f"provenance:         {provenance}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
