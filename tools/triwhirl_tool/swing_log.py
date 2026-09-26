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
    "source_vertex_id",
    "source_vertex_center_deg",
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


def _local_error_rad(theta_rad: float, center_deg: float) -> float:
    center_rad = math.radians(center_deg)
    return math.atan2(
        math.sin(theta_rad - center_rad),
        math.cos(theta_rad - center_rad),
    )


def _fit_row(
    *,
    trial: int,
    source_vertex_id: str,
    source_center_deg: float,
    model_center_deg: float,
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

    # All legal upright passages are the same local control coordinate. Preserve
    # the physical source orientation as provenance, but normalize the actual fit
    # signal to one canonical upright reference so the downstream legacy fitter
    # pools every autonomous crossing into one plant instead of inventing A/B/C
    # controller identities.
    local_error = _local_error_rad(theta_rad, source_center_deg)
    model_ref_rad = math.radians(model_center_deg)
    normalized_theta_rad = model_ref_rad + local_error

    return (
        4,
        trial,
        "A",  # compatibility label for the existing fitter; not a control identity
        model_center_deg,
        source_vertex_id,
        source_center_deg,
        phase,
        model_ref_rad,
        planned_vq_v,
        t_us,
        normalized_theta_rad,
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
    model_center_deg = centers["A"]
    rows: list[tuple[object, ...]] = []
    trial = 0
    index = 0

    while index < len(records):
        flags = int(records[index][7])
        if not (flags & RECORD_PROBE_ACTIVE):
            index += 1
            continue

        start = index
        source_vertex_id = _vertex_from_flags(flags)
        while index < len(records) and (int(records[index][7]) & RECORD_PROBE_ACTIVE):
            current_vertex = _vertex_from_flags(int(records[index][7]))
            if current_vertex != source_vertex_id:
                raise RuntimeError(
                    "vertex flag changed inside one contiguous probe window: "
                    f"{source_vertex_id}->{current_vertex}"
                )
            index += 1
        stop = index

        trial += 1
        source_center_deg = centers[source_vertex_id]
        planned_vq_v = float(records[start][4])
        probe_phase = "zero_vector" if abs(planned_vq_v) <= 1.0e-6 else "active"

        # Keep one pre-probe record as the fitter's kinematic anchor. Native swing
        # ID can change from coarse pump to the probe schedule at Probe entry; the
        # fitter rejects derivative windows that cross that Vq transition.
        if start > 0:
            rows.append(
                _fit_row(
                    trial=trial,
                    source_vertex_id=source_vertex_id,
                    source_center_deg=source_center_deg,
                    model_center_deg=model_center_deg,
                    phase="armed",
                    planned_vq_v=planned_vq_v,
                    record=records[start - 1],
                )
            )

        for probe_index in range(start, stop):
            rows.append(
                _fit_row(
                    trial=trial,
                    source_vertex_id=source_vertex_id,
                    source_center_deg=source_center_deg,
                    model_center_deg=model_center_deg,
                    phase=probe_phase,
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
    sources = sorted({str(row[4]) for row in rows})
    print(
        "saved local-normalized fit-ready swing windows: "
        f"{trials} trials, {len(rows)} rows, source orientations={','.join(sources)}"
    )
    print(output_path)
    return trials
