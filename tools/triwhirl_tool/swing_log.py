from __future__ import annotations

import csv
import math
from pathlib import Path

from . import twlog

RECORD_CRITICAL_WINDOW = 1 << 9
RECORD_VERTEX_A = 1 << 10
RECORD_VERTEX_B = 1 << 11
RECORD_VERTEX_C = 1 << 12
RECORD_PROBE_ACTIVE = 1 << 13
RECORD_PUMP_ACTIVE = 1 << 14

VERTEX_BITS = {
    "A": RECORD_VERTEX_A,
    "B": RECORD_VERTEX_B,
    "C": RECORD_VERTEX_C,
}

FIT_FIELDS = (
    "schema_version",
    "trial",
    "vertex_id",
    "vertex_center_deg",
    "phase",
    "theta_ref_rad",
    "planned_vq_v",
    "t_us",
    "theta_rad",
    "theta_rate_rad_s",
    "vel_rad_s",
    "vq_v",
)


def wrap_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def vertex_centers_deg(vertex_a_deg: float) -> dict[str, float]:
    return {
        "A": wrap_deg(vertex_a_deg),
        "B": wrap_deg(vertex_a_deg - 120.0),
        "C": wrap_deg(vertex_a_deg + 120.0),
    }


def _vertex_from_flags(flags: int) -> str:
    found = [vertex_id for vertex_id, bit in VERTEX_BITS.items() if flags & bit]
    if len(found) != 1:
        raise RuntimeError(
            "probe record must carry exactly one VertexA/B/C flag; "
            f"flags=0x{flags:04x}"
        )
    return found[0]


def _fit_row(
    *,
    trial: int,
    vertex_id: str,
    center_deg: float,
    phase: str,
    planned_vq_v: float,
    record: tuple[int, float, float, float, float, float, int, int, int],
) -> tuple[object, ...]:
    (
        t_us,
        theta_rad,
        theta_rate_rad_s,
        wheel_rate_rad_s,
        vq_v,
        _accel_weight,
        _fault_mask,
        _flags,
        _raw_count,
    ) = record
    return (
        3,
        trial,
        vertex_id,
        center_deg,
        phase,
        math.radians(center_deg),
        planned_vq_v,
        t_us,
        theta_rad,
        theta_rate_rad_s,
        wheel_rate_rad_s,
        vq_v,
    )


def extract_fit_rows(
    payload: bytes,
    *,
    vertex_a_deg: float = 68.0,
) -> list[tuple[object, ...]]:
    if not math.isfinite(vertex_a_deg):
        raise ValueError("vertex_a_deg must be finite")

    records = list(twlog.iter_records(payload))
    centers = vertex_centers_deg(vertex_a_deg)
    rows: list[tuple[object, ...]] = []
    trial = 0
    index = 0

    while index < len(records):
        flags = int(records[index][7])
        if not (flags & RECORD_PROBE_ACTIVE):
            index += 1
            continue

        start = index
        vertex_id = _vertex_from_flags(flags)
        while index < len(records) and (int(records[index][7]) & RECORD_PROBE_ACTIVE):
            current_vertex = _vertex_from_flags(int(records[index][7]))
            if current_vertex != vertex_id:
                raise RuntimeError(
                    "vertex flag changed inside one contiguous probe window: "
                    f"{vertex_id}->{current_vertex}"
                )
            index += 1
        stop = index

        trial += 1
        center_deg = centers[vertex_id]
        planned_vq_v = float(records[start][4])

        # Keep one pre-probe record as the fitter's kinematic anchor.  The
        # on-device runner freezes the already-established pump Vq at Probe
        # entry, so this sample should normally carry the same signed input.
        if start > 0:
            rows.append(
                _fit_row(
                    trial=trial,
                    vertex_id=vertex_id,
                    center_deg=center_deg,
                    phase="armed",
                    planned_vq_v=planned_vq_v,
                    record=records[start - 1],
                )
            )

        for probe_index in range(start, stop):
            rows.append(
                _fit_row(
                    trial=trial,
                    vertex_id=vertex_id,
                    center_deg=center_deg,
                    phase="active",
                    planned_vq_v=planned_vq_v,
                    record=records[probe_index],
                )
            )

    if trial == 0:
        raise RuntimeError("TWLG contains no ProbeActive swing-identification windows")
    return rows


def write_fit_csv(
    input_path: Path,
    output_path: Path,
    *,
    vertex_a_deg: float = 68.0,
) -> int:
    _meta, payload = twlog.read(input_path)
    rows = extract_fit_rows(payload, vertex_a_deg=vertex_a_deg)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIT_FIELDS)
        writer.writerows(rows)
    trials = len({int(row[1]) for row in rows})
    print(f"saved fit-ready swing windows: {trials} trials, {len(rows)} rows")
    print(output_path)
    return trials
