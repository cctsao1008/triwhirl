from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Sequence

from ..plant_replay import replay, write_replay_csv, write_summary


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
    active = {key: value for key, value in limits.items() if value is not None}
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
        f"{name}={measured[name]:.6g} > {limit:.6g}"
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


def _replay_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a fitted TriWhirl local linear plant against measured Vq and "
            "compare one-step and free-run trajectories."
        )
    )
    parser.add_argument("model", type=Path, help="linear-model JSON from from_active_fit.py")
    parser.add_argument("data", type=Path, help="active-ID or standup CSV")
    parser.add_argument(
        "--plant",
        default="auto",
        help="plant selection: auto, nominal, A, B, or C",
    )
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument(
        "--no-bias",
        action="store_true",
        help="ignore affine fit bias during replay",
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, default=None, help="optional sample-by-sample replay CSV")
    _add_parity_limits(parser)
    return parser


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


def replay_main(argv: Sequence[str]) -> int:
    args = _replay_parser().parse_args(list(argv))
    try:
        summary, samples = replay(
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
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f"error: {exc}")
        return 1

    _print_summary(summary)
    print(f"parity_json={args.output}")
    if args.csv is not None:
        print(f"parity_csv={args.csv}")
    return 0 if passed is not False else 3


def _calibrate_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit per-vertex local plants, build the shared linear model, and run "
            "replay parity. A separate --validation CSV is required before parity "
            "can be treated as a synthesis gate."
        )
    )
    parser.add_argument("input", type=Path, help="fit-ready body-active CSV")
    parser.add_argument(
        "--validation",
        type=Path,
        default=None,
        help="independent fit-ready CSV for holdout replay parity",
    )
    parser.add_argument("--vertices", default="auto")
    parser.add_argument("--nominal", default="mean")
    parser.add_argument("--derivative-window", type=int, default=3)
    parser.add_argument("--vertex-a-deg", type=float, default=68.0)
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument("--plant", default="auto")
    parser.add_argument("--no-bias", action="store_true")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/plant-calibration"),
    )
    parser.add_argument("--provenance-note", default="")
    _add_parity_limits(parser)
    return parser


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run(command: list[str]) -> None:
    print("+ " + " ".join(command))
    subprocess.run(command, check=True)


def _evaluate_fit_gate(linear_path: Path) -> dict[str, object]:
    payload = json.loads(linear_path.read_text(encoding="utf-8"))
    if payload.get("format") != "triwhirl-vertex-normalized-linear-model-v1":
        raise RuntimeError("linear model uses an unsupported format")

    selected = payload.get("selected_vertices")
    plants = payload.get("plants")
    nominal = payload.get("nominal")
    if not isinstance(selected, list) or not selected:
        raise RuntimeError("linear model selected no plant vertices")
    if not isinstance(plants, list) or not plants:
        raise RuntimeError("linear model contains no plant vertices")
    if not isinstance(nominal, dict):
        raise RuntimeError("linear model contains no nominal plant")

    issues: list[str] = []
    plant_checks: list[dict[str, object]] = []
    by_vertex = {
        str(item.get("vertex_id", "")).upper(): item
        for item in plants
        if isinstance(item, dict)
    }
    for raw_vertex in selected:
        vertex = str(raw_vertex).upper()
        plant = by_vertex.get(vertex)
        if plant is None:
            issues.append(f"selected vertex {vertex} has no linear plant")
            continue

        source_status = str(plant.get("source_status", "unknown"))
        rank = int(plant.get("controllability_rank", 0))
        eig = plant.get("open_loop_eigenvalues", [])
        max_real = None
        if isinstance(eig, list) and eig:
            real_parts = [
                float(item.get("real"))
                for item in eig
                if isinstance(item, dict) and item.get("real") is not None
            ]
            if real_parts and all(math.isfinite(value) for value in real_parts):
                max_real = max(real_parts)

        plant_issues: list[str] = []
        if source_status != "candidate":
            plant_issues.append(f"source_status={source_status}; expected candidate")
        if rank != 3:
            plant_issues.append(f"controllability_rank={rank}; expected 3")
        if max_real is None:
            plant_issues.append("open-loop eigenvalues are missing or non-finite")
        elif max_real <= 0.0:
            plant_issues.append(
                f"no open-loop unstable mode detected (max real eigenvalue={max_real:.6g})"
            )

        if plant_issues:
            issues.extend(f"vertex {vertex}: {item}" for item in plant_issues)
        plant_checks.append(
            {
                "vertex_id": vertex,
                "source_status": source_status,
                "controllability_rank": rank,
                "max_open_loop_real_eigenvalue": max_real,
                "status": "PASS" if not plant_issues else "FAIL",
                "issues": plant_issues,
            }
        )

    nominal_rank = int(nominal.get("controllability_rank", 0))
    nominal_eig = nominal.get("open_loop_eigenvalues", [])
    nominal_real = [
        float(item.get("real"))
        for item in nominal_eig
        if isinstance(item, dict) and item.get("real") is not None
    ] if isinstance(nominal_eig, list) else []
    nominal_max_real = (
        max(nominal_real)
        if nominal_real and all(math.isfinite(value) for value in nominal_real)
        else None
    )
    if nominal_rank != 3:
        issues.append(f"nominal controllability_rank={nominal_rank}; expected 3")
    if nominal_max_real is None:
        issues.append("nominal open-loop eigenvalues are missing or non-finite")
    elif nominal_max_real <= 0.0:
        issues.append(
            "nominal plant has no open-loop unstable mode "
            f"(max real eigenvalue={nominal_max_real:.6g})"
        )

    return {
        "status": "PASS" if not issues else "FAIL",
        "selected_vertices": [str(value).upper() for value in selected],
        "plants": plant_checks,
        "nominal_controllability_rank": nominal_rank,
        "nominal_max_open_loop_real_eigenvalue": nominal_max_real,
        "issues": issues,
    }


def calibrate_main(argv: Sequence[str]) -> int:
    args = _calibrate_parser().parse_args(list(argv))
    repo = Path(__file__).resolve().parents[3]
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    fit_path = output_dir / "active-fit.json"
    linear_path = output_dir / "linear-model.json"
    parity_path = output_dir / "replay-parity.json"
    replay_csv = output_dir / "replay-parity.csv"
    manifest_path = output_dir / "calibration-manifest.json"

    fit_command = [
        sys.executable,
        str(repo / "tools/parameter_id/body_active_fit.py"),
        str(args.input),
        "--derivative-window",
        str(args.derivative_window),
        "--max-angle-deg",
        str(args.max_angle_deg),
        "--vertex-a-deg",
        str(args.vertex_a_deg),
        "-o",
        str(fit_path),
    ]
    linear_command = [
        sys.executable,
        str(repo / "model/linearization/from_active_fit.py"),
        str(fit_path),
        "--vertices",
        args.vertices,
        "--nominal",
        args.nominal,
        "--provenance-note",
        args.provenance_note,
        "-o",
        str(linear_path),
    ]

    try:
        _run(fit_command)
        _run(linear_command)
        fit_gate = _evaluate_fit_gate(linear_path)
        replay_source = args.validation if args.validation is not None else args.input
        summary, samples = replay(
            linear_path,
            replay_source,
            plant_selection=args.plant,
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
            summary["synthesis_gate"] = {
                "status": "BLOCKED",
                "reason": "one or more selected plant fits failed the synthesis fit gate",
            }
        elif args.validation is None:
            summary["synthesis_gate"] = {
                "status": "BLOCKED",
                "reason": (
                    "parity is in-sample; collect an independent validation dataset "
                    "before promoting this plant to H-infinity synthesis"
                ),
            }
        elif passed is False:
            summary["synthesis_gate"] = {
                "status": "BLOCKED",
                "reason": "holdout replay parity thresholds failed",
            }
        elif passed is None:
            summary["synthesis_gate"] = {
                "status": "REVIEW",
                "reason": "holdout replay exists but no quantitative thresholds were supplied",
            }
        else:
            summary["synthesis_gate"] = {
                "status": "PASS",
                "reason": (
                    "candidate/full-rank unstable local plants and independent holdout replay "
                    "met the supplied parity thresholds"
                ),
            }

        write_summary(parity_path, summary)
        write_replay_csv(replay_csv, samples)

        artifact_sha256 = {
            "fit": _sha256(fit_path),
            "linear_model": _sha256(linear_path),
            "parity": _sha256(parity_path),
            "replay_csv": _sha256(replay_csv),
        }
        manifest = {
            "format": "triwhirl-plant-calibration-v1",
            "calibration_input": str(args.input),
            "calibration_sha256": _sha256(args.input),
            "validation_input": str(args.validation) if args.validation is not None else None,
            "validation_sha256": _sha256(args.validation) if args.validation is not None else None,
            "validation_mode": validation_mode,
            "vertices": args.vertices,
            "nominal": args.nominal,
            "max_angle_deg": args.max_angle_deg,
            "derivative_window": args.derivative_window,
            "vertex_a_deg": args.vertex_a_deg,
            "include_affine_bias_in_replay": not args.no_bias,
            "parity_acceptance": summary.get("acceptance"),
            "fit_gate": fit_gate,
            "artifacts": {
                "fit": fit_path.name,
                "linear_model": linear_path.name,
                "parity": parity_path.name,
                "replay_csv": replay_csv.name,
            },
            "artifact_sha256": artifact_sha256,
            "synthesis_gate": summary["synthesis_gate"],
            "provenance_note": args.provenance_note,
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    except (OSError, RuntimeError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
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
    print(f"synthesis_gate={summary['synthesis_gate']['status']}")
    if args.validation is None:
        print("NEXT_COLLECT_INDEPENDENT_VALIDATION_DATASET")
        return 0
    if summary["synthesis_gate"]["status"] == "BLOCKED":
        return 3
    return 0
