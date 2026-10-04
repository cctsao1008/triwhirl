from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Sequence

from ..local_plant_replay import model_format, replay_local
from ..plant_replay import replay as replay_legacy
from ..plant_replay import write_replay_csv, write_summary


LOCAL_MODEL_FORMAT = "triwhirl-local-linear-model-v1"
LEGACY_MODEL_FORMAT = "triwhirl-vertex-normalized-linear-model-v1"


def _add_parity_limits(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--max-theta-rollout-rmse-deg", type=float, default=None)
    parser.add_argument("--max-rate-rollout-rmse", type=float, default=None)
    parser.add_argument("--max-wheel-rollout-rmse", type=float, default=None)


def _apply_acceptance(summary: dict[str, object], args: argparse.Namespace) -> bool | None:
    limits = {
        "theta_rollout_rmse_deg": args.max_theta_rollout_rmse_deg,
        "rate_rollout_rmse_rad_s": args.max_rate_rollout_rmse,
        "wheel_rollout_rmse_rad_s": args.max_wheel_rollout_rmse,
    }
    active = {name: value for name, value in limits.items() if value is not None}
    if not active:
        summary["acceptance"] = {
            "status": "UNSET",
            "reason": "no parity thresholds supplied",
            "limits": limits,
            "violations": [],
        }
        return None

    metrics = summary["metrics"]
    measured = {
        "theta_rollout_rmse_deg": metrics["theta_error"]["rollout_rmse_deg"],
        "rate_rollout_rmse_rad_s": metrics["theta_rate"]["rollout_rmse"],
        "wheel_rollout_rmse_rad_s": metrics["wheel_rate"]["rollout_rmse"],
    }
    violations = [
        f"{name}={float(measured[name]):.6g} > {float(limit):.6g}"
        for name, limit in active.items()
        if float(measured[name]) > float(limit)
    ]
    passed = not violations
    summary["acceptance"] = {
        "status": "PASS" if passed else "FAIL",
        "limits": limits,
        "measured": measured,
        "violations": violations,
    }
    return passed


def _print_summary(summary: dict[str, object]) -> None:
    metrics = summary["metrics"]
    print(
        "parity,"
        f"segments={summary['segment_count']},"
        f"intervals={summary['interval_count']},"
        f"theta_rollout_rmse_deg={metrics['theta_error']['rollout_rmse_deg']:.6f},"
        f"rate_rollout_rmse={metrics['theta_rate']['rollout_rmse']:.6f},"
        f"wheel_rollout_rmse={metrics['wheel_rate']['rollout_rmse']:.6f},"
        f"theta_one_step_rmse_deg={metrics['theta_error']['one_step_rmse_deg']:.6f}"
    )
    acceptance = summary.get("acceptance", {})
    if isinstance(acceptance, dict):
        print(f"parity_acceptance={acceptance.get('status', 'UNSET')}")
        for violation in acceptance.get("violations", []):
            print(f"parity_violation={violation}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run(command: list[str]) -> None:
    print("+ " + " ".join(command))
    subprocess.run(command, check=True)


def _replay_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay either the current vertex-agnostic local model or a historical "
            "vertex-normalized model against measured Vq."
        )
    )
    parser.add_argument("model", type=Path)
    parser.add_argument("data", type=Path)
    parser.add_argument(
        "--plant",
        default="auto",
        help=(
            "current local models accept auto/nominal; historical models also "
            "accept A, B, or C"
        ),
    )
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument("--no-bias", action="store_true")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, default=None)
    _add_parity_limits(parser)
    return parser


def _replay_for_model(
    model: Path,
    data: Path,
    *,
    plant_selection: str,
    max_angle_deg: float,
    include_bias: bool,
) -> tuple[dict[str, object], list[object]]:
    fmt = model_format(model)
    if fmt == LOCAL_MODEL_FORMAT:
        if plant_selection.strip().upper() not in {"AUTO", "NOMINAL"}:
            raise RuntimeError(
                "vertex-agnostic local models accept only --plant auto or nominal"
            )
        return replay_local(
            model,
            data,
            max_angle_deg=max_angle_deg,
            include_bias=include_bias,
        )
    if fmt == LEGACY_MODEL_FORMAT:
        return replay_legacy(
            model,
            data,
            plant_selection=plant_selection,
            max_angle_deg=max_angle_deg,
            include_bias=include_bias,
        )
    raise RuntimeError(f"unsupported linear-model format: {fmt!r}")


def replay_main(argv: Sequence[str]) -> int:
    args = _replay_parser().parse_args(list(argv))
    try:
        summary, samples = _replay_for_model(
            args.model,
            args.data,
            plant_selection=args.plant,
            max_angle_deg=args.max_angle_deg,
            include_bias=not args.no_bias,
        )
        passed = _apply_acceptance(summary, args)
        write_summary(args.output, summary)
        if args.csv is not None:
            write_replay_csv(args.csv, samples)
    except (OSError, RuntimeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"error: {exc}")
        return 1

    _print_summary(summary)
    print(f"model_format={model_format(args.model)}")
    print(f"parity_json={args.output}")
    if args.csv is not None:
        print(f"parity_csv={args.csv}")
    return 0 if passed is not False else 3


def _calibrate_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit the current vertex-agnostic near-upright plant, build the local "
            "linear model, and evaluate independent holdout replay parity."
        )
    )
    parser.add_argument("input", type=Path, help="body-active local calibration CSV")
    parser.add_argument(
        "--validation",
        type=Path,
        default=None,
        help="independent body-active local CSV used only for holdout replay",
    )
    parser.add_argument("--derivative-window", type=int, default=3)
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument("--min-trials", type=int, default=2)
    parser.add_argument("--min-active-samples-per-sign", type=int, default=6)
    parser.add_argument("--min-theta-samples-per-sign", type=int, default=4)
    parser.add_argument("--min-vq-coef-sigma", type=float, default=2.0)
    parser.add_argument("--no-bias", action="store_true")
    parser.add_argument(
        "--output-dir", type=Path, default=Path("artifacts/plant-calibration")
    )
    parser.add_argument("--provenance-note", default="")
    _add_parity_limits(parser)
    return parser


def _evaluate_local_fit_gate(linear_path: Path) -> dict[str, object]:
    payload = json.loads(linear_path.read_text(encoding="utf-8"))
    if payload.get("format") != LOCAL_MODEL_FORMAT:
        raise RuntimeError("linear model uses an unsupported current format")
    nominal = payload.get("nominal")
    if not isinstance(nominal, dict):
        raise RuntimeError("local linear model contains no nominal plant")

    issues: list[str] = []
    source_status = str(nominal.get("source_status", "unknown"))
    rank = int(nominal.get("controllability_rank", 0))
    eigenvalues = nominal.get("open_loop_eigenvalues", [])
    real_parts: list[float] = []
    if isinstance(eigenvalues, list):
        for item in eigenvalues:
            if isinstance(item, dict) and item.get("real") is not None:
                value = float(item["real"])
                if math.isfinite(value):
                    real_parts.append(value)
    max_real = max(real_parts) if real_parts else None

    if source_status != "candidate":
        issues.append(f"source_status={source_status}; expected candidate")
    if rank != 3:
        issues.append(f"controllability_rank={rank}; expected 3")
    if max_real is None:
        issues.append("open-loop eigenvalues are missing or non-finite")
    elif max_real <= 0.0:
        issues.append(
            f"no open-loop unstable upright mode detected (max real={max_real:.6g})"
        )

    return {
        "status": "PASS" if not issues else "FAIL",
        "model_format": LOCAL_MODEL_FORMAT,
        "source_status": source_status,
        "controllability_rank": rank,
        "max_open_loop_real_eigenvalue": max_real,
        "issues": issues,
    }


def calibrate_main(argv: Sequence[str]) -> int:
    args = _calibrate_parser().parse_args(list(argv))
    repo = Path(__file__).resolve().parents[3]
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    fit_path = output_dir / "active-local-fit.json"
    linear_path = output_dir / "linear-model.json"
    parity_path = output_dir / "replay-parity.json"
    replay_csv = output_dir / "replay-parity.csv"
    manifest_path = output_dir / "calibration-manifest.json"

    fit_command = [
        sys.executable,
        str(repo / "tools/parameter_id/body_active_local_fit.py"),
        str(args.input),
        "--derivative-window",
        str(args.derivative_window),
        "--max-angle-deg",
        str(args.max_angle_deg),
        "--min-trials",
        str(args.min_trials),
        "--min-active-samples-per-sign",
        str(args.min_active_samples_per_sign),
        "--min-theta-samples-per-sign",
        str(args.min_theta_samples_per_sign),
        "--min-vq-coef-sigma",
        str(args.min_vq_coef_sigma),
        "-o",
        str(fit_path),
    ]
    linear_command = [
        sys.executable,
        str(repo / "model/linearization/from_local_fit.py"),
        str(fit_path),
        "--provenance-note",
        args.provenance_note,
        "-o",
        str(linear_path),
    ]

    try:
        _run(fit_command)
        _run(linear_command)
        fit_gate = _evaluate_local_fit_gate(linear_path)
        replay_source = args.validation if args.validation is not None else args.input
        summary, samples = replay_local(
            linear_path,
            replay_source,
            max_angle_deg=args.max_angle_deg,
            include_bias=not args.no_bias,
        )
        passed = _apply_acceptance(summary, args)
        validation_mode = (
            "external_holdout" if args.validation is not None else "in_sample_diagnostic"
        )
        summary["validation_mode"] = validation_mode
        summary["fit_gate"] = fit_gate

        if fit_gate["status"] != "PASS":
            promotion_gate = {
                "status": "BLOCKED",
                "reason": "the calibration fit/model gate failed",
            }
        elif args.validation is None:
            promotion_gate = {
                "status": "BLOCKED",
                "reason": "collect an independent validation dataset before controller tuning",
            }
        elif passed is False:
            promotion_gate = {
                "status": "BLOCKED",
                "reason": "holdout replay parity thresholds failed",
            }
        elif passed is None:
            promotion_gate = {
                "status": "REVIEW",
                "reason": "holdout replay exists but empirical parity thresholds are unset",
            }
        else:
            promotion_gate = {
                "status": "PASS",
                "reason": (
                    "candidate local model and independent holdout replay met the "
                    "explicit supplied parity thresholds"
                ),
            }
        summary["promotion_gate"] = promotion_gate

        write_summary(parity_path, summary)
        write_replay_csv(replay_csv, samples)
        artifact_sha256 = {
            "fit": _sha256(fit_path),
            "linear_model": _sha256(linear_path),
            "parity": _sha256(parity_path),
            "replay_csv": _sha256(replay_csv),
        }
        manifest = {
            "format": "triwhirl-plant-calibration-v2",
            "authority": "LOCAL_MODEL_EVIDENCE_ONLY",
            "coordinate_contract": (
                "Per trial theta_error_rad = wrap(theta_rad - theta_ref_rad); "
                "absolute theta_ref_rad is provenance only."
            ),
            "calibration_input": str(args.input),
            "calibration_sha256": _sha256(args.input),
            "validation_input": str(args.validation) if args.validation is not None else None,
            "validation_sha256": _sha256(args.validation) if args.validation is not None else None,
            "validation_mode": validation_mode,
            "model_format": LOCAL_MODEL_FORMAT,
            "max_angle_deg": args.max_angle_deg,
            "derivative_window": args.derivative_window,
            "include_affine_bias_in_replay": not args.no_bias,
            "parity_acceptance": summary.get("acceptance"),
            "fit_gate": fit_gate,
            "promotion_gate": promotion_gate,
            "artifacts": {
                "fit": fit_path.name,
                "linear_model": linear_path.name,
                "parity": parity_path.name,
                "replay_csv": replay_csv.name,
            },
            "artifact_sha256": artifact_sha256,
            "provenance_note": args.provenance_note,
            "legacy_hinf_compatible": False,
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    except (
        OSError,
        RuntimeError,
        ValueError,
        KeyError,
        json.JSONDecodeError,
        subprocess.CalledProcessError,
    ) as exc:
        print(f"error: {exc}")
        return 1

    _print_summary(summary)
    print(f"fit_gate={fit_gate['status']}")
    for issue in fit_gate.get("issues", []):
        print(f"fit_gate_issue={issue}")
    print(f"calibration_manifest={manifest_path}")
    print(f"fit={fit_path}")
    print(f"linear_model={linear_path}")
    print(f"parity={parity_path}")
    print(f"promotion_gate={promotion_gate['status']}")
    if args.validation is None:
        print("NEXT_COLLECT_INDEPENDENT_VALIDATION_DATASET")
        return 0
    if promotion_gate["status"] == "BLOCKED":
        return 3
    return 0
