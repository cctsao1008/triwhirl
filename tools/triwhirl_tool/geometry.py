from __future__ import annotations

import math
from dataclasses import dataclass

VERTEX_IDS = ("A", "B", "C")


@dataclass(frozen=True)
class VertexMatch:
    vertex_id: str
    center_deg: float
    error_deg: float


def wrap_deg(angle_deg: float) -> float:
    """Wrap degrees to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def angle_diff_deg(angle_deg: float, reference_deg: float) -> float:
    """Signed circular difference angle-reference in degrees."""
    return wrap_deg(angle_deg - reference_deg)


def vertex_centers_deg(vertex_a_deg: float = 68.0) -> dict[str, float]:
    """Return the three nominal upright centers in the current IMU frame."""
    return {
        "A": wrap_deg(vertex_a_deg),
        "B": wrap_deg(vertex_a_deg - 120.0),
        "C": wrap_deg(vertex_a_deg + 120.0),
    }


def classify_vertex_deg(
    theta_deg: float,
    vertex_a_deg: float = 68.0,
    tolerance_deg: float = 20.0,
) -> VertexMatch | None:
    """Classify an upright angle as A/B/C, or None if it is between vertices."""
    if not math.isfinite(theta_deg) or not math.isfinite(vertex_a_deg):
        return None
    if not math.isfinite(tolerance_deg) or tolerance_deg <= 0.0 or tolerance_deg >= 60.0:
        raise ValueError("vertex tolerance must be finite, > 0 and < 60 degrees")

    centers = vertex_centers_deg(vertex_a_deg)
    best_id = min(
        VERTEX_IDS,
        key=lambda vertex_id: abs(angle_diff_deg(theta_deg, centers[vertex_id])),
    )
    error = angle_diff_deg(theta_deg, centers[best_id])
    if abs(error) > tolerance_deg:
        return None
    return VertexMatch(best_id, centers[best_id], error)
