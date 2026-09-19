#!/usr/bin/env python3
"""Fit actively excited near-upright TriWhirl dynamics per upright vertex.

TriWhirl has three legitimate upright equilibria.  They are geometrically 120
degrees apart, but the real PCB/battery/motor mass distribution need not be
perfectly three-fold symmetric.  Therefore this fitter classifies every trial
as vertex A/B/C and fits each vertex separately; it never pools different
vertices into one controller plant merely because their local coordinates can
be normalized.

Body model per vertex:
    theta_ddot = a_theta * theta_error_gyro
               + a_rate  * theta_rate
               + a_wheel * wheel_rate
               + b_vq    * vq
               + bias

Wheel model per vertex:
    wheel_accel = c_theta * theta_error_gyro
                + c_rate  * theta_rate
                + c_wheel * wheel_rate
                + d_vq    * vq
                + bias

``theta_error_gyro`` is integrated from the calibrated gyro starting at the
last armed sample before release.  The measured firmware ``vq_v`` is the
identification input.  Older active CSV files without explicit ``vertex_id``
columns are still supported by classifying their held ``theta_ref_rad``.

A full-rank least-squares result is not enough to call a vertex model a
controller candidate.  Candidate acceptance also requires useful two-sided
local-angle coverage and statistically separated Vq gains in both body and
wheel equations.  This prevents a numerically clean but physically weak fit
from being promoted into controller synthesis.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from vertex_geometry import VERTEX_IDS, classify_vertex_deg, vertex_centers_deg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit active near-upright plant dynamics separately for A/B/C vertices."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("--derivative-window", type=int, default=1)
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument(
        "--vertex-a-deg", type=float, default=68.0,
        help="nominal A-vertex angle; B/C are -120/+120 deg from A",
    )
    parser.add_argument("--vertex-tolerance-deg", type=float, default=20.0)
    parser.add_argument(
        "--min-active-samples-per-sign", type=int, default=6,
        help="minimum held-input regression samples for +Vq and -Vq within one vertex",
    )
    parser.add_argument(
        "--min-theta-samples-per-sign", type=int, default=4,
        help="minimum usable gyro-integrated theta-error samples on each side of zero",
    )
    parser.add_argument(
        "--min-vq-coef-sigma", type=float, default=2.0,
        help="minimum |Vq coefficient|/standard-error for both body and wheel equations",
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def local_slope(t: np.ndarray, y: np.ndarray, center: int, half: int) -> float:
    lo = center - half
    hi = center + half + 1
    tt = t[lo:hi]
    yy = y[lo:hi]
    tc = tt - np.mean(tt)
    denom = float(np.dot(tc, tc))
    if denom <= 0.0:
        return float("nan")
    return float(np.dot(tc, yy - np.mean(yy)) / denom)


def normalized_condition_number(x: np.ndarray) -> float:
    if x.shape[1] <= 1:
        return float("inf")
    physical = x[:, :-1]
    scale = np.std(physical, axis=0)
    keep = scale > 1.0e-12
    if not np.any(keep):
        return float("inf")
    xn = (physical[:, keep] - np.mean(physical[:, keep], axis=0)) / scale[keep]
    return float(np.linalg.cond(xn))


def fit_equation(x: np.ndarray, y: np.ndarray, names: list[str]) -> dict[str, object]:
    beta, _residuals, rank, singular = np.linalg.lstsq(x, y, rcond=None)
    y_hat = x @ beta
    resid = y - y_hat
    sse = float(np.dot(resid, resid))
    sst = float(np.dot(y - np.mean(y), y - np.mean(y)))
    r2 = 1.0 - sse / sst if sst > 0.0 else 0.0
    rmse = float(np.sqrt(np.mean(np.square(resid))))

    dof = len(y) - int(rank)
    if dof > 0 and int(rank) == x.shape[1]:
        sigma2 = sse / dof
        covariance = sigma2 * np.linalg.inv(x.T @ x)
        stderr = np.sqrt(np.diag(covariance))
    else:
        stderr = np.full(x.shape[1], np.nan)

    coefficients: dict[str, object] = {}
    for i, name in enumerate(names):
        se = None if not np.isfinite(stderr[i]) else float(stderr[i])
        ratio = None if se is None or se == 0.0 else float(abs(beta[i]) / se)
        coefficients[name] = {
            "value": float(beta[i]),
            "std_error": se,
            "abs_over_std_error": ratio,
        }

    return {
        "rank": int(rank),
        "rmse": rmse,
        "r_squared": float(r2),
        "singular_values": [float(v) for v in singular],
        "coefficients": coefficients,
    }


def classify_trial(
    trial_rows: list[dict[str, str]],
    vertex_a_deg: float,
    tolerance_deg: float,
) -> tuple[str | None, dict[str, object]]:
    refs = [float(r["theta_ref_rad"]) for r in trial_rows if r.get("theta_ref_rad")]
    if not refs:
        return None, {"reason": "missing_theta_ref"}
    ref_rad = float(np.median(refs))
    ref_deg = math.degrees(ref_rad)
    match = classify_vertex_deg(ref_deg, vertex_a_deg, tolerance_deg)
    if match is None:
        return None, {
            "reason": "not_near_any_vertex",
            "theta_ref_rad": ref_rad,
            "theta_ref_deg": ref_deg,
        }

    explicit_ids = {
        r.get("vertex_id", "").strip()
        for r in trial_rows
        if r.get("vertex_id", "").strip()
    }
    explicit_ids.discard("")
    if explicit_ids and explicit_ids != {match.vertex_id}:
        return None, {
            "reason": "vertex_id_disagrees_with_theta_ref",
            "theta_ref_rad": ref_rad,
            "theta_ref_deg": ref_deg,
            "classified_vertex": match.vertex_id,
            "explicit_vertex_ids": sorted(explicit_ids),
        }

    return match.vertex_id, {
        "theta_ref_rad": ref_rad,
        "theta_ref_deg": ref_deg,
        "vertex_id": match.vertex_id,
        "vertex_center_deg": match.center_deg,
        "vertex_error_deg": match.error_deg,
    }


def extract_trial_samples(
    trial: int,
    tr: list[dict[str, str]],
    derivative_window: int,
    max_angle: float,
) -> tuple[list[list[float]], list[float], list[float], dict[str, object], int, int]:
    tr = sorted(tr, key=lambda r: int(r["t_us"]))
    dynamic_indices = [
        i for i, r in enumerate(tr)
        if r.get("phase") in ("active", "zero_vector")
    ]
    if not dynamic_indices:
        return [], [], [], {"trial": trial, "used": 0, "reason": "no_dynamic_rows"}, 0, 0

    first_dynamic = dynamic_indices[0]
    armed_before = [i for i in range(first_dynamic) if tr[i].get("phase") == "armed"]
    anchor_i = armed_before[-1] if armed_before else max(0, first_dynamic - 1)
    dyn = tr[anchor_i: dynamic_indices[-1] + 1]

    t = np.asarray([float(r["t_us"]) * 1.0e-6 for r in dyn])
    rate = np.asarray([float(r["theta_rate_rad_s"]) for r in dyn])
    wheel = np.asarray([float(r["vel_rad_s"]) for r in dyn])
    vq = np.asarray([float(r["vq_v"]) for r in dyn])
    theta = np.asarray([float(r["theta_rad"]) for r in dyn])
    theta_ref = float(dyn[0]["theta_ref_rad"])

    error = np.empty(len(dyn), dtype=float)
    error[0] = angle_diff(theta[0], theta_ref)
    for i in range(1, len(dyn)):
        dt = t[i] - t[i - 1]
        if dt <= 0.0:
            error[i] = error[i - 1]
        else:
            error[i] = error[i - 1] + 0.5 * (rate[i - 1] + rate[i]) * dt

    regressors: list[list[float]] = []
    body_targets: list[float] = []
    wheel_targets: list[float] = []
    active_used = 0
    zero_used = 0
    rejected_transition = 0
    rejected_angle = 0

    for i in range(derivative_window, len(dyn) - derivative_window):
        if dyn[i].get("phase") not in ("active", "zero_vector"):
            continue
        lo = i - derivative_window
        hi = i + derivative_window + 1
        if not np.all(np.diff(t[lo:hi]) > 0.0):
            continue
        if abs(error[i]) > max_angle:
            rejected_angle += 1
            continue
        if float(np.max(vq[lo:hi]) - np.min(vq[lo:hi])) > 1.0e-6:
            rejected_transition += 1
            continue

        body_accel = local_slope(t, rate, i, derivative_window)
        wheel_accel = local_slope(t, wheel, i, derivative_window)
        if not np.isfinite(body_accel) or not np.isfinite(wheel_accel):
            continue

        regressors.append([error[i], rate[i], wheel[i], vq[i], 1.0])
        body_targets.append(body_accel)
        wheel_targets.append(wheel_accel)
        if abs(vq[i]) > 1.0e-9:
            active_used += 1
        else:
            zero_used += 1

    meta = {
        "trial": trial,
        "theta_ref_rad": theta_ref,
        "theta_ref_deg": math.degrees(theta_ref),
        "planned_vq_v": float(tr[first_dynamic].get("planned_vq_v", "nan")),
        "dynamic_rows": len(dynamic_indices),
        "used": len(regressors),
        "active_used": active_used,
        "zero_vector_used": zero_used,
        "theta_error_min_deg": math.degrees(float(np.min(error))),
        "theta_error_max_deg": math.degrees(float(np.max(error))),
        "measured_vq_min_v": float(np.min(vq)),
        "measured_vq_max_v": float(np.max(vq)),
    }
    return (
        regressors,
        body_targets,
        wheel_targets,
        meta,
        rejected_transition,
        rejected_angle,
    )


def main() -> int:
    args = parse_args()
    if args.derivative_window < 1:
        raise RuntimeError("--derivative-window must be >= 1")
    if args.vertex_tolerance_deg <= 0.0 or args.vertex_tolerance_deg >= 60.0:
        raise RuntimeError("--vertex-tolerance-deg must be > 0 and < 60")
    if args.min_active_samples_per_sign < 1:
        raise RuntimeError("--min-active-samples-per-sign must be >= 1")
    if args.min_theta_samples_per_sign < 1:
        raise RuntimeError("--min-theta-samples-per-sign must be >= 1")
    if args.min_vq_coef_sigma <= 0.0:
        raise RuntimeError("--min-vq-coef-sigma must be > 0")

    rows = read_rows(args.input)
    if not rows:
        raise RuntimeError("input CSV is empty")

    names = [
        "theta_error_gyro_rad", "theta_rate_rad_s", "wheel_rate_rad_s",
        "vq_v", "bias",
    ]
    max_angle = math.radians(args.max_angle_deg)
    trial_ids = sorted({int(r["trial"]) for r in rows if r.get("trial")})
    groups: dict[str, dict[str, object]] = {
        vertex_id: {
            "regressors": [],
            "body_targets": [],
            "wheel_targets": [],
            "trials": [],
            "rejected_transition": 0,
            "rejected_angle": 0,
        }
        for vertex_id in VERTEX_IDS
    }
    unclassified_trials: list[dict[str, object]] = []

    for trial in trial_ids:
        tr = [r for r in rows if int(r.get("trial", "0")) == trial]
        vertex_id, classification = classify_trial(
            tr, args.vertex_a_deg, args.vertex_tolerance_deg
        )
        if vertex_id is None:
            unclassified_trials.append({"trial": trial, **classification})
            continue

        x_rows, y_body, y_wheel, meta, reject_transition, reject_angle = (
            extract_trial_samples(
                trial, tr, args.derivative_window, max_angle
            )
        )
        meta.update(classification)
        group = groups[vertex_id]
        group["regressors"].extend(x_rows)
        group["body_targets"].extend(y_body)
        group["wheel_targets"].extend(y_wheel)
        group["trials"].append(meta)
        group["rejected_transition"] += reject_transition
        group["rejected_angle"] += reject_angle

    vertex_fits: dict[str, object] = {}
    candidate_vertices: list[str] = []

    for vertex_id in VERTEX_IDS:
        group = groups[vertex_id]
        regressors = group["regressors"]
        trial_meta = group["trials"]
        center_deg = vertex_centers_deg(args.vertex_a_deg)[vertex_id]
        base: dict[str, object] = {
            "vertex_id": vertex_id,
            "nominal_center_deg": center_deg,
            "trials": trial_meta,
            "sample_count": len(regressors),
            "rejected_windows": {
                "vq_transition": group["rejected_transition"],
                "angle_limit": group["rejected_angle"],
            },
        }

        if len(regressors) < 5:
            base.update({
                "status": "insufficient_data",
                "reasons": [f"only {len(regressors)} usable samples"],
            })
            vertex_fits[vertex_id] = base
            continue

        x = np.asarray(regressors, dtype=float)
        y_body = np.asarray(group["body_targets"], dtype=float)
        y_wheel = np.asarray(group["wheel_targets"], dtype=float)
        vq_values = x[:, 3]
        theta_values = x[:, 0]
        positive_vq = int(np.sum(vq_values > 1.0e-9))
        negative_vq = int(np.sum(vq_values < -1.0e-9))
        positive_theta = int(np.sum(theta_values > 0.0))
        negative_theta = int(np.sum(theta_values < 0.0))
        reasons: list[str] = []
        if len(x) < 12:
            reasons.append(f"only {len(x)} usable samples; need at least 12")
        if positive_vq < args.min_active_samples_per_sign:
            reasons.append(
                f"positive Vq samples={positive_vq}; need {args.min_active_samples_per_sign}"
            )
        if negative_vq < args.min_active_samples_per_sign:
            reasons.append(
                f"negative Vq samples={negative_vq}; need {args.min_active_samples_per_sign}"
            )
        if positive_theta < args.min_theta_samples_per_sign:
            reasons.append(
                f"positive theta-error samples={positive_theta}; need {args.min_theta_samples_per_sign}"
            )
        if negative_theta < args.min_theta_samples_per_sign:
            reasons.append(
                f"negative theta-error samples={negative_theta}; need {args.min_theta_samples_per_sign}"
            )

        body_fit = fit_equation(x, y_body, names)
        wheel_fit = fit_equation(x, y_wheel, names)
        if body_fit["rank"] < 5 or wheel_fit["rank"] < 5:
            reasons.append("regression is not full rank")

        body_vq_sigma = body_fit["coefficients"]["vq_v"]["abs_over_std_error"]
        wheel_vq_sigma = wheel_fit["coefficients"]["vq_v"]["abs_over_std_error"]
        if body_vq_sigma is None or body_vq_sigma < args.min_vq_coef_sigma:
            reasons.append(
                "body Vq gain is not statistically separated: "
                f"|coef|/SE={body_vq_sigma if body_vq_sigma is not None else 'n/a'}; "
                f"need {args.min_vq_coef_sigma:.2f}"
            )
        if wheel_vq_sigma is None or wheel_vq_sigma < args.min_vq_coef_sigma:
            reasons.append(
                "wheel Vq gain is not statistically separated: "
                f"|coef|/SE={wheel_vq_sigma if wheel_vq_sigma is not None else 'n/a'}; "
                f"need {args.min_vq_coef_sigma:.2f}"
            )

        status = "candidate" if not reasons else "diagnostic_only"
        if status == "candidate":
            candidate_vertices.append(vertex_id)

        base.update({
            "status": status,
            "reasons": reasons,
            "condition_number_raw": float(np.linalg.cond(x)),
            "condition_number_normalized": normalized_condition_number(x),
            "input_coverage": {
                "vq_min_v": float(np.min(vq_values)),
                "vq_max_v": float(np.max(vq_values)),
                "positive_vq_samples": positive_vq,
                "negative_vq_samples": negative_vq,
                "zero_vq_samples": int(np.sum(np.abs(vq_values) <= 1.0e-9)),
                "positive_theta_error_samples": positive_theta,
                "negative_theta_error_samples": negative_theta,
            },
            "acceptance_metrics": {
                "body_vq_abs_over_std_error": body_vq_sigma,
                "wheel_vq_abs_over_std_error": wheel_vq_sigma,
                "min_vq_coef_sigma": args.min_vq_coef_sigma,
                "min_theta_samples_per_sign": args.min_theta_samples_per_sign,
            },
            "body_equation": body_fit,
            "wheel_equation": wheel_fit,
            "candidate_A_rows": {
                "body": {
                    "theta_error": body_fit["coefficients"]["theta_error_gyro_rad"]["value"],
                    "theta_rate": body_fit["coefficients"]["theta_rate_rad_s"]["value"],
                    "wheel_rate": body_fit["coefficients"]["wheel_rate_rad_s"]["value"],
                },
                "wheel": {
                    "theta_error": wheel_fit["coefficients"]["theta_error_gyro_rad"]["value"],
                    "theta_rate": wheel_fit["coefficients"]["theta_rate_rad_s"]["value"],
                    "wheel_rate": wheel_fit["coefficients"]["wheel_rate_rad_s"]["value"],
                },
            },
            "candidate_B_v": {
                "body": body_fit["coefficients"]["vq_v"]["value"],
                "wheel": wheel_fit["coefficients"]["vq_v"]["value"],
            },
        })
        vertex_fits[vertex_id] = base

    payload = {
        "format": "triwhirl-body-active-fit-v4",
        "input": str(args.input),
        "model": {
            "body": (
                "theta_ddot = a_theta*theta_error_gyro + a_rate*theta_rate + "
                "a_wheel*wheel_rate + b_vq*vq + bias"
            ),
            "wheel": (
                "wheel_accel = c_theta*theta_error_gyro + c_rate*theta_rate + "
                "c_wheel*wheel_rate + d_vq*vq + bias"
            ),
        },
        "vertex_geometry": {
            "vertex_a_deg": args.vertex_a_deg,
            "vertex_tolerance_deg": args.vertex_tolerance_deg,
            "centers_deg": vertex_centers_deg(args.vertex_a_deg),
            "note": (
                "A/B/C labels are IMU-frame naming anchors. Fits remain separate because "
                "real mass distribution may break ideal three-fold symmetry."
            ),
        },
        "derivative_window_each_side": args.derivative_window,
        "max_angle_deg": args.max_angle_deg,
        "min_active_samples_per_sign": args.min_active_samples_per_sign,
        "min_theta_samples_per_sign": args.min_theta_samples_per_sign,
        "min_vq_coef_sigma": args.min_vq_coef_sigma,
        "unclassified_trials": unclassified_trials,
        "candidate_vertices": candidate_vertices,
        "vertex_fits": vertex_fits,
        "interpretation": (
            "Never pool different upright vertices automatically. Candidate status requires "
            "two-sided local-angle coverage, signed-Vq coverage, full-rank regression, and "
            "statistically separated body/wheel Vq gains; otherwise the fit is retained as "
            "diagnostic plant evidence only."
        ),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print(args.output)
    centers = vertex_centers_deg(args.vertex_a_deg)
    print(
        "vertex centers: "
        + ", ".join(f"{vertex_id}={center:.1f} deg" for vertex_id, center in centers.items())
    )
    for vertex_id in VERTEX_IDS:
        fit = vertex_fits[vertex_id]
        print(
            f"vertex {vertex_id}: status={fit['status']} samples={fit['sample_count']} "
            f"trials={len(fit['trials'])}"
        )
        if fit.get("reasons"):
            for reason in fit["reasons"]:
                print(f"  - {reason}")
        if "body_equation" in fit:
            print(
                f"  body: R2={fit['body_equation']['r_squared']:.4f} "
                f"RMSE={fit['body_equation']['rmse']:.4f}"
            )
            print(
                f"  wheel: R2={fit['wheel_equation']['r_squared']:.4f} "
                f"RMSE={fit['wheel_equation']['rmse']:.4f}"
            )
            coverage = fit["input_coverage"]
            metrics = fit["acceptance_metrics"]
            print(
                f"  Vq samples: +={coverage['positive_vq_samples']} "
                f"-={coverage['negative_vq_samples']} zero={coverage['zero_vq_samples']}"
            )
            print(
                f"  theta-error samples: +={coverage['positive_theta_error_samples']} "
                f"-={coverage['negative_theta_error_samples']}"
            )
            print(
                f"  Vq |coef|/SE: body={metrics['body_vq_abs_over_std_error']:.2f} "
                f"wheel={metrics['wheel_vq_abs_over_std_error']:.2f}"
            )
    if unclassified_trials:
        print("unclassified/non-vertex trials:")
        for item in unclassified_trials:
            print(f"  trial {item['trial']}: {item['reason']}")
    if candidate_vertices:
        print("candidate vertex models: " + ", ".join(candidate_vertices))
    else:
        print("no vertex model is synthesis-ready yet; retained fits are diagnostic evidence")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
