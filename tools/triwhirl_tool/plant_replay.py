from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


_STATE_NAMES = ("theta_error_rad", "theta_rate_rad_s", "wheel_rate_rad_s")


@dataclass(frozen=True)
class LinearPlant:
    name: str
    A: tuple[tuple[float, float, float], ...]
    B: tuple[float, float, float]
    bias: tuple[float, float, float]
    vertex_id: str | None = None
    nominal_center_deg: float | None = None


@dataclass(frozen=True)
class ReplaySample:
    segment: str
    plant: str
    t_s: float
    dt_s: float
    u_v: float
    measured: tuple[float, float, float]
    one_step: tuple[float, float, float]
    rollout: tuple[float, float, float]


@dataclass(frozen=True)
class ReplaySegment:
    segment: str
    plant: str
    samples: tuple[ReplaySample, ...]


def _vec_add(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_scale(a: Sequence[float], s: float) -> tuple[float, float, float]:
    return (a[0] * s, a[1] * s, a[2] * s)


def _derivative(
    plant: LinearPlant,
    state: Sequence[float],
    u_v: float,
    include_bias: bool,
) -> tuple[float, float, float]:
    out = []
    for row_i in range(3):
        row = plant.A[row_i]
        value = (
            row[0] * state[0]
            + row[1] * state[1]
            + row[2] * state[2]
            + plant.B[row_i] * u_v
        )
        if include_bias:
            value += plant.bias[row_i]
        out.append(value)
    return (out[0], out[1], out[2])


def rk4_step(
    plant: LinearPlant,
    state: Sequence[float],
    u_v: float,
    dt_s: float,
    *,
    include_bias: bool = True,
) -> tuple[float, float, float]:
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and > 0")
    if dt_s > 0.1:
        raise ValueError("dt_s exceeds 100 ms replay safety limit")

    k1 = _derivative(plant, state, u_v, include_bias)
    k2 = _derivative(
        plant,
        _vec_add(state, _vec_scale(k1, 0.5 * dt_s)),
        u_v,
        include_bias,
    )
    k3 = _derivative(
        plant,
        _vec_add(state, _vec_scale(k2, 0.5 * dt_s)),
        u_v,
        include_bias,
    )
    k4 = _derivative(
        plant,
        _vec_add(state, _vec_scale(k3, dt_s)),
        u_v,
        include_bias,
    )
    weighted = (
        k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0],
        k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1],
        k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2],
    )
    return _vec_add(state, _vec_scale(weighted, dt_s / 6.0))


def _matrix3(value: object, name: str) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError(f"{name} must be a 3x3 list")
    rows: list[tuple[float, float, float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 3:
            raise RuntimeError(f"{name} must be a 3x3 list")
        rows.append((float(row[0]), float(row[1]), float(row[2])))
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
    return (flat[0], flat[1], flat[2])


def load_linear_model(path: Path) -> tuple[LinearPlant, dict[str, LinearPlant]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("format") != "triwhirl-vertex-normalized-linear-model-v1":
        raise RuntimeError("unsupported linear-model format")
    if payload.get("state") != list(_STATE_NAMES):
        raise RuntimeError("linear model uses an unexpected state order")
    if payload.get("input") != "Vq_v":
        raise RuntimeError("linear model input must be Vq_v")

    plants_raw = payload.get("plants")
    if not isinstance(plants_raw, list) or not plants_raw:
        raise RuntimeError("linear model has no per-vertex plants")

    plants: dict[str, LinearPlant] = {}
    for item in plants_raw:
        if not isinstance(item, dict):
            raise RuntimeError("invalid plant entry")
        vertex = str(item.get("vertex_id", "")).upper()
        if vertex not in {"A", "B", "C"}:
            raise RuntimeError(f"invalid plant vertex_id: {vertex!r}")
        plants[vertex] = LinearPlant(
            name=f"vertex_{vertex}",
            A=_matrix3(item.get("A"), f"plants[{vertex}].A"),
            B=_vector3(item.get("Bv"), f"plants[{vertex}].Bv"),
            bias=_vector3(item.get("affine_bias"), f"plants[{vertex}].affine_bias"),
            vertex_id=vertex,
            nominal_center_deg=float(item.get("nominal_center_deg", 0.0)),
        )

    nominal_raw = payload.get("nominal")
    if not isinstance(nominal_raw, dict):
        raise RuntimeError("linear model has no nominal plant")
    nominal = LinearPlant(
        name="nominal",
        A=_matrix3(nominal_raw.get("A"), "nominal.A"),
        B=_vector3(nominal_raw.get("Bv"), "nominal.Bv"),
        bias=_vector3(nominal_raw.get("affine_bias"), "nominal.affine_bias"),
    )
    return nominal, plants


def _wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _nearest_vertex(theta_ref_rad: float, plants: dict[str, LinearPlant]) -> str | None:
    theta_deg = math.degrees(theta_ref_rad)
    candidates: list[tuple[float, str]] = []
    for vertex, plant in plants.items():
        if plant.nominal_center_deg is None:
            continue
        candidates.append((abs(_wrap_deg(theta_deg - plant.nominal_center_deg)), vertex))
    if not candidates:
        return None
    distance, vertex = min(candidates)
    return vertex if distance < 60.0 else None


def _float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if value == "":
        raise RuntimeError(f"missing CSV field {key}")
    return float(value)


def _integrate_error_from_rate(rows: list[dict[str, str]], anchor: int) -> list[float]:
    error = [0.0] * len(rows)
    theta0 = _float(rows[anchor], "theta_rad")
    ref0 = _float(rows[anchor], "theta_ref_rad")
    error[anchor] = math.atan2(math.sin(theta0 - ref0), math.cos(theta0 - ref0))
    for i in range(anchor + 1, len(rows)):
        t0 = _float(rows[i - 1], "t_us") * 1.0e-6
        t1 = _float(rows[i], "t_us") * 1.0e-6
        dt = t1 - t0
        if dt <= 0.0:
            error[i] = error[i - 1]
            continue
        r0 = _float(rows[i - 1], "theta_rate_rad_s")
        r1 = _float(rows[i], "theta_rate_rad_s")
        error[i] = error[i - 1] + 0.5 * (r0 + r1) * dt
    return error


def _active_segments(rows: list[dict[str, str]], max_angle_rad: float) -> list[dict[str, object]]:
    trial_ids = sorted({int(row["trial"]) for row in rows if row.get("trial")})
    segments: list[dict[str, object]] = []
    for trial in trial_ids:
        tr = [row for row in rows if int(row.get("trial", "0")) == trial]
        tr.sort(key=lambda row: int(row["t_us"]))
        dynamic = [
            i for i, row in enumerate(tr)
            if row.get("phase") in {"active", "zero_vector"}
        ]
        if len(dynamic) < 2:
            continue
        first = dynamic[0]
        armed = [i for i in range(first) if tr[i].get("phase") == "armed"]
        anchor = armed[-1] if armed else max(0, first - 1)
        errors = _integrate_error_from_rate(tr, anchor)
        samples = []
        for i in dynamic:
            if abs(errors[i]) > max_angle_rad:
                continue
            samples.append(
                {
                    "t_s": _float(tr[i], "t_us") * 1.0e-6,
                    "error": errors[i],
                    "rate": _float(tr[i], "theta_rate_rad_s"),
                    "wheel": _float(tr[i], "vel_rad_s"),
                    "vq": _float(tr[i], "vq_v"),
                }
            )
        if len(samples) < 2:
            continue
        explicit = {
            row.get("vertex_id", "").strip().upper()
            for row in tr
            if row.get("vertex_id", "").strip()
        }
        vertex_id = next(iter(explicit)) if len(explicit) == 1 else None
        theta_refs = [
            _float(row, "theta_ref_rad")
            for row in tr
            if row.get("theta_ref_rad", "") != ""
        ]
        theta_ref = theta_refs[len(theta_refs) // 2] if theta_refs else None
        segments.append(
            {
                "name": f"trial_{trial}",
                "vertex_id": vertex_id,
                "theta_ref_rad": theta_ref,
                "samples": samples,
            }
        )
    return segments


def _standup_segments(rows: list[dict[str, str]], max_angle_rad: float) -> list[dict[str, object]]:
    segments: list[dict[str, object]] = []
    current: list[dict[str, float]] = []
    segment_index = 0

    def flush() -> None:
        nonlocal current, segment_index
        if len(current) >= 2:
            segments.append(
                {
                    "name": f"balance_{segment_index}",
                    "vertex_id": None,
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
    flush()
    return segments


def read_replay_segments(path: Path, max_angle_deg: float) -> tuple[str, list[dict[str, object]]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("replay CSV is empty")
    fields = set(rows[0])
    max_angle_rad = math.radians(max_angle_deg)
    if {"trial", "t_us", "theta_ref_rad", "vel_rad_s", "vq_v"}.issubset(fields):
        return "active_id", _active_segments(rows, max_angle_rad)
    if {"t_s", "error_deg", "wheel_rate_rad_s", "vq_applied_v", "phase"}.issubset(fields):
        return "standup", _standup_segments(rows, max_angle_rad)
    raise RuntimeError("unsupported replay CSV schema")


def _select_plant(
    segment: dict[str, object],
    nominal: LinearPlant,
    plants: dict[str, LinearPlant],
    selection: str,
) -> LinearPlant:
    spec = selection.strip().upper()
    if spec == "NOMINAL":
        return nominal
    if spec in plants:
        return plants[spec]
    if spec != "AUTO":
        raise RuntimeError("--plant must be auto, nominal, A, B, or C")

    vertex = segment.get("vertex_id")
    if isinstance(vertex, str) and vertex in plants:
        return plants[vertex]
    theta_ref = segment.get("theta_ref_rad")
    if isinstance(theta_ref, (float, int)):
        nearest = _nearest_vertex(float(theta_ref), plants)
        if nearest is not None:
            return plants[nearest]
    return nominal


def _rmse(values: Iterable[float]) -> float:
    seq = list(values)
    if not seq:
        return float("nan")
    return math.sqrt(sum(v * v for v in seq) / len(seq))


def _mae(values: Iterable[float]) -> float:
    seq = list(values)
    if not seq:
        return float("nan")
    return sum(abs(v) for v in seq) / len(seq)


def _range(values: Iterable[float]) -> float:
    seq = list(values)
    return max(seq) - min(seq) if seq else 0.0


def _metrics(samples: Sequence[ReplaySample], attr: str) -> dict[str, object]:
    index = {"theta": 0, "rate": 1, "wheel": 2}[attr]
    measured = [sample.measured[index] for sample in samples]
    one = [sample.one_step[index] - sample.measured[index] for sample in samples]
    rollout = [sample.rollout[index] - sample.measured[index] for sample in samples]
    scale = _range(measured)
    one_rmse = _rmse(one)
    roll_rmse = _rmse(rollout)
    return {
        "one_step_rmse": one_rmse,
        "one_step_mae": _mae(one),
        "rollout_rmse": roll_rmse,
        "rollout_mae": _mae(rollout),
        "signal_range": scale,
        "one_step_nrmse_range": one_rmse / scale if scale > 1.0e-12 else None,
        "rollout_nrmse_range": roll_rmse / scale if scale > 1.0e-12 else None,
    }


def replay(
    model_path: Path,
    data_path: Path,
    *,
    plant_selection: str = "auto",
    max_angle_deg: float = 8.0,
    include_bias: bool = True,
) -> tuple[dict[str, object], list[ReplaySample]]:
    if max_angle_deg <= 0.0 or max_angle_deg >= 60.0:
        raise ValueError("max_angle_deg must be > 0 and < 60")
    nominal, plants = load_linear_model(model_path)
    schema, raw_segments = read_replay_segments(data_path, max_angle_deg)
    if not raw_segments:
        raise RuntimeError("no replayable local segments found")

    segments: list[ReplaySegment] = []
    all_samples: list[ReplaySample] = []
    for raw in raw_segments:
        points = raw["samples"]
        if not isinstance(points, list) or len(points) < 2:
            continue
        plant = _select_plant(raw, nominal, plants, plant_selection)
        first = points[0]
        rollout_state = (
            float(first["error"]),
            float(first["rate"]),
            float(first["wheel"]),
        )
        segment_samples: list[ReplaySample] = []
        for i in range(1, len(points)):
            prev = points[i - 1]
            cur = points[i]
            dt = float(cur["t_s"]) - float(prev["t_s"])
            if dt <= 0.0 or dt > 0.1:
                continue
            prev_measured = (
                float(prev["error"]),
                float(prev["rate"]),
                float(prev["wheel"]),
            )
            measured = (
                float(cur["error"]),
                float(cur["rate"]),
                float(cur["wheel"]),
            )
            u_v = float(prev["vq"])
            one_step = rk4_step(
                plant, prev_measured, u_v, dt, include_bias=include_bias
            )
            rollout_state = rk4_step(
                plant, rollout_state, u_v, dt, include_bias=include_bias
            )
            sample = ReplaySample(
                segment=str(raw["name"]),
                plant=plant.name,
                t_s=float(cur["t_s"]),
                dt_s=dt,
                u_v=u_v,
                measured=measured,
                one_step=one_step,
                rollout=rollout_state,
            )
            segment_samples.append(sample)
            all_samples.append(sample)
        if segment_samples:
            segments.append(
                ReplaySegment(
                    segment=str(raw["name"]),
                    plant=plant.name,
                    samples=tuple(segment_samples),
                )
            )

    if not all_samples:
        raise RuntimeError("no valid replay intervals found")

    theta = _metrics(all_samples, "theta")
    theta["one_step_rmse_deg"] = math.degrees(float(theta["one_step_rmse"]))
    theta["rollout_rmse_deg"] = math.degrees(float(theta["rollout_rmse"]))
    theta["signal_range_deg"] = math.degrees(float(theta["signal_range"]))

    segment_summaries = []
    for segment in segments:
        segment_theta = _metrics(segment.samples, "theta")
        segment_theta["one_step_rmse_deg"] = math.degrees(
            float(segment_theta["one_step_rmse"])
        )
        segment_theta["rollout_rmse_deg"] = math.degrees(
            float(segment_theta["rollout_rmse"])
        )
        segment_summaries.append(
            {
                "segment": segment.segment,
                "plant": segment.plant,
                "interval_count": len(segment.samples),
                "theta_error": segment_theta,
                "theta_rate": _metrics(segment.samples, "rate"),
                "wheel_rate": _metrics(segment.samples, "wheel"),
            }
        )

    summary: dict[str, object] = {
        "format": "triwhirl-replay-parity-v1",
        "model": str(model_path),
        "data": str(data_path),
        "data_schema": schema,
        "plant_selection": plant_selection,
        "include_affine_bias": include_bias,
        "max_angle_deg": max_angle_deg,
        "segment_count": len(segments),
        "interval_count": len(all_samples),
        "metrics": {
            "theta_error": theta,
            "theta_rate": _metrics(all_samples, "rate"),
            "wheel_rate": _metrics(all_samples, "wheel"),
        },
        "segments": segment_summaries,
    }
    return summary, all_samples


def write_replay_csv(path: Path, samples: Sequence[ReplaySample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "segment",
        "plant",
        "t_s",
        "dt_s",
        "vq_v",
        "theta_measured_rad",
        "theta_one_step_rad",
        "theta_rollout_rad",
        "rate_measured_rad_s",
        "rate_one_step_rad_s",
        "rate_rollout_rad_s",
        "wheel_measured_rad_s",
        "wheel_one_step_rad_s",
        "wheel_rollout_rad_s",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "segment": sample.segment,
                    "plant": sample.plant,
                    "t_s": f"{sample.t_s:.9f}",
                    "dt_s": f"{sample.dt_s:.9f}",
                    "vq_v": f"{sample.u_v:.9f}",
                    "theta_measured_rad": f"{sample.measured[0]:.9f}",
                    "theta_one_step_rad": f"{sample.one_step[0]:.9f}",
                    "theta_rollout_rad": f"{sample.rollout[0]:.9f}",
                    "rate_measured_rad_s": f"{sample.measured[1]:.9f}",
                    "rate_one_step_rad_s": f"{sample.one_step[1]:.9f}",
                    "rate_rollout_rad_s": f"{sample.rollout[1]:.9f}",
                    "wheel_measured_rad_s": f"{sample.measured[2]:.9f}",
                    "wheel_one_step_rad_s": f"{sample.one_step[2]:.9f}",
                    "wheel_rollout_rad_s": f"{sample.rollout[2]:.9f}",
                }
            )


def write_summary(path: Path, summary: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
