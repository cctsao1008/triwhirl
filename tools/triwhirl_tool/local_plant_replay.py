from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

from .plant_replay import LinearPlant, ReplaySample, rk4_step, write_replay_csv, write_summary


_STATE_NAMES = ("theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s")
_LOCAL_FORMAT = "triwhirl-local-linear-model-v1"


def _matrix3(value: object, name: str) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError(f"{name} must be a 3x3 list")
    rows: list[tuple[float, float, float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 3:
            raise RuntimeError(f"{name} must be a 3x3 list")
        converted = tuple(float(item) for item in row)
        if not all(math.isfinite(item) for item in converted):
            raise RuntimeError(f"{name} contains non-finite values")
        rows.append(converted)
    return tuple(rows)


def _vector3(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError(f"{name} must have 3 entries")
    flat: list[float] = []
    for item in value:
        if isinstance(item, list):
            if len(item) != 1:
                raise RuntimeError(f"{name} nested entries must be scalar")
            flat.append(float(item[0]))
        else:
            flat.append(float(item))
    if not all(math.isfinite(item) for item in flat):
        raise RuntimeError(f"{name} contains non-finite values")
    return (flat[0], flat[1], flat[2])


def load_local_linear_model(path: Path) -> LinearPlant:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("format") != _LOCAL_FORMAT:
        raise RuntimeError("unsupported vertex-agnostic local linear-model format")
    if payload.get("state") != list(_STATE_NAMES):
        raise RuntimeError("linear model uses an unexpected state order")
    if payload.get("input") != "Vq_v":
        raise RuntimeError("linear model input must be Vq_v")
    if payload.get("active_error_mode") != "wrapped_theta_ref":
        raise RuntimeError("local linear model must use wrapped_theta_ref active error mode")

    nominal = payload.get("nominal")
    if not isinstance(nominal, dict):
        raise RuntimeError("local linear model has no nominal plant")
    return LinearPlant(
        name="local_nominal",
        A=_matrix3(nominal.get("A"), "nominal.A"),
        B=_vector3(nominal.get("Bv"), "nominal.Bv"),
        bias=_vector3(nominal.get("affine_bias"), "nominal.affine_bias"),
    )


def model_format(path: Path) -> str:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return str(payload.get("format", ""))


def _float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if value == "":
        raise RuntimeError(f"missing CSV field {key}")
    converted = float(value)
    if not math.isfinite(converted):
        raise RuntimeError(f"non-finite CSV field {key}")
    return converted


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _active_segments(
    rows: list[dict[str, str]], max_angle_rad: float
) -> list[dict[str, object]]:
    trial_ids = sorted({int(row["trial"]) for row in rows if row.get("trial")})
    segments: list[dict[str, object]] = []
    for trial in trial_ids:
        trial_rows = [row for row in rows if int(row.get("trial", "0")) == trial]
        trial_rows.sort(key=lambda row: int(row["t_us"]))
        dynamic = [
            row
            for row in trial_rows
            if row.get("phase") in {"active", "zero_vector"}
        ]
        samples: list[dict[str, float]] = []
        refs: list[float] = []
        for row in dynamic:
            try:
                theta_ref = _float(row, "theta_ref_rad")
                error = _wrap(_float(row, "theta_rad") - theta_ref)
                if abs(error) > max_angle_rad:
                    continue
                refs.append(theta_ref)
                samples.append(
                    {
                        "t_s": _float(row, "t_us") * 1.0e-6,
                        "error": error,
                        "rate": _float(row, "theta_rate_rad_s"),
                        "wheel": _float(row, "vel_rad_s"),
                        "vq": _float(row, "vq_v"),
                    }
                )
            except RuntimeError:
                continue
        if len(samples) >= 2:
            refs_sorted = sorted(refs)
            theta_ref = refs_sorted[len(refs_sorted) // 2] if refs_sorted else None
            segments.append(
                {
                    "name": f"trial_{trial}",
                    "theta_ref_rad": theta_ref,
                    "samples": samples,
                }
            )
    return segments


def _standup_segments(
    rows: list[dict[str, str]], max_angle_rad: float
) -> list[dict[str, object]]:
    segments: list[dict[str, object]] = []
    current: list[dict[str, float]] = []
    segment_index = 0

    def flush() -> None:
        nonlocal current, segment_index
        if len(current) >= 2:
            segments.append(
                {
                    "name": f"balance_{segment_index}",
                    "theta_ref_rad": None,
                    "samples": current,
                }
            )
            segment_index += 1
        current = []

    for row in rows:
        if row.get("phase") != "balance":
            flush()
            continue
        try:
            error = math.radians(_float(row, "error_deg"))
            if abs(error) > max_angle_rad:
                flush()
                continue
            current.append(
                {
                    "t_s": _float(row, "t_s"),
                    "error": error,
                    "rate": _float(row, "theta_rate_rad_s"),
                    "wheel": _float(row, "wheel_rate_rad_s"),
                    "vq": _float(row, "vq_applied_v"),
                }
            )
        except RuntimeError:
            flush()
    flush()
    return segments


def read_local_replay_segments(
    path: Path, max_angle_deg: float
) -> tuple[str, list[dict[str, object]]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("replay CSV is empty")
    max_angle_rad = math.radians(max_angle_deg)
    fields = set(rows[0])
    if {
        "trial",
        "t_us",
        "theta_ref_rad",
        "theta_rad",
        "theta_rate_rad_s",
        "vel_rad_s",
        "vq_v",
    }.issubset(fields):
        return "active_id_local", _active_segments(rows, max_angle_rad)
    if {"t_s", "error_deg", "wheel_rate_rad_s", "vq_applied_v", "phase"}.issubset(fields):
        return "standup", _standup_segments(rows, max_angle_rad)
    raise RuntimeError("unsupported replay CSV schema")


def _rmse(values: Iterable[float]) -> float:
    sequence = list(values)
    if not sequence:
        return float("nan")
    return math.sqrt(sum(value * value for value in sequence) / len(sequence))


def _mae(values: Iterable[float]) -> float:
    sequence = list(values)
    if not sequence:
        return float("nan")
    return sum(abs(value) for value in sequence) / len(sequence)


def _range(values: Iterable[float]) -> float:
    sequence = list(values)
    return max(sequence) - min(sequence) if sequence else 0.0


def _metrics(samples: Sequence[ReplaySample], index: int) -> dict[str, object]:
    measured = [sample.measured[index] for sample in samples]
    one_step_error = [
        sample.one_step[index] - sample.measured[index] for sample in samples
    ]
    rollout_error = [
        sample.rollout[index] - sample.measured[index] for sample in samples
    ]
    scale = _range(measured)
    one_rmse = _rmse(one_step_error)
    rollout_rmse = _rmse(rollout_error)
    return {
        "one_step_rmse": one_rmse,
        "one_step_mae": _mae(one_step_error),
        "rollout_rmse": rollout_rmse,
        "rollout_mae": _mae(rollout_error),
        "signal_range": scale,
        "one_step_nrmse_range": one_rmse / scale if scale > 1.0e-12 else None,
        "rollout_nrmse_range": rollout_rmse / scale if scale > 1.0e-12 else None,
    }


def replay_local(
    model_path: Path,
    data_path: Path,
    *,
    max_angle_deg: float = 8.0,
    include_bias: bool = True,
) -> tuple[dict[str, object], list[ReplaySample]]:
    if max_angle_deg <= 0.0 or max_angle_deg >= 60.0:
        raise ValueError("max_angle_deg must be > 0 and < 60")
    plant = load_local_linear_model(model_path)
    schema, raw_segments = read_local_replay_segments(data_path, max_angle_deg)
    if not raw_segments:
        raise RuntimeError("no replayable local segments found")

    all_samples: list[ReplaySample] = []
    segment_summaries: list[dict[str, object]] = []
    for raw in raw_segments:
        points = raw.get("samples")
        if not isinstance(points, list) or len(points) < 2:
            continue
        first = points[0]
        rollout_state = (
            float(first["error"]),
            float(first["rate"]),
            float(first["wheel"]),
        )
        segment_samples: list[ReplaySample] = []
        for index in range(1, len(points)):
            previous = points[index - 1]
            current = points[index]
            dt_s = float(current["t_s"]) - float(previous["t_s"])
            if dt_s <= 0.0 or dt_s > 0.1:
                continue
            previous_measured = (
                float(previous["error"]),
                float(previous["rate"]),
                float(previous["wheel"]),
            )
            measured = (
                float(current["error"]),
                float(current["rate"]),
                float(current["wheel"]),
            )
            u_v = float(previous["vq"])
            one_step = rk4_step(
                plant, previous_measured, u_v, dt_s, include_bias=include_bias
            )
            rollout_state = rk4_step(
                plant, rollout_state, u_v, dt_s, include_bias=include_bias
            )
            sample = ReplaySample(
                segment=str(raw["name"]),
                plant=plant.name,
                t_s=float(current["t_s"]),
                dt_s=dt_s,
                u_v=u_v,
                measured=measured,
                one_step=one_step,
                rollout=rollout_state,
            )
            segment_samples.append(sample)
            all_samples.append(sample)
        if segment_samples:
            theta = _metrics(segment_samples, 0)
            theta["one_step_rmse_deg"] = math.degrees(float(theta["one_step_rmse"]))
            theta["rollout_rmse_deg"] = math.degrees(float(theta["rollout_rmse"]))
            segment_summaries.append(
                {
                    "segment": str(raw["name"]),
                    "plant": plant.name,
                    "theta_ref_rad": raw.get("theta_ref_rad"),
                    "interval_count": len(segment_samples),
                    "theta_error": theta,
                    "theta_rate": _metrics(segment_samples, 1),
                    "wheel_rate": _metrics(segment_samples, 2),
                }
            )

    if not all_samples:
        raise RuntimeError("no valid replay intervals found")

    theta = _metrics(all_samples, 0)
    theta["one_step_rmse_deg"] = math.degrees(float(theta["one_step_rmse"]))
    theta["rollout_rmse_deg"] = math.degrees(float(theta["rollout_rmse"]))
    theta["signal_range_deg"] = math.degrees(float(theta["signal_range"]))
    summary: dict[str, object] = {
        "format": "triwhirl-replay-parity-v1",
        "model_format": _LOCAL_FORMAT,
        "model": str(model_path),
        "data": str(data_path),
        "data_schema": schema,
        "active_error_mode": "wrapped_theta_ref",
        "plant_selection": "nominal",
        "include_affine_bias": include_bias,
        "max_angle_deg": max_angle_deg,
        "segment_count": len(segment_summaries),
        "interval_count": len(all_samples),
        "metrics": {
            "theta_error": theta,
            "theta_rate": _metrics(all_samples, 1),
            "wheel_rate": _metrics(all_samples, 2),
        },
        "segments": segment_summaries,
    }
    return summary, all_samples


__all__ = [
    "load_local_linear_model",
    "model_format",
    "read_local_replay_segments",
    "replay_local",
    "write_replay_csv",
    "write_summary",
]
