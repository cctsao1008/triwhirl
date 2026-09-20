#!/usr/bin/env python3
"""Compatibility wrapper for shared TriWhirl vertex geometry helpers."""

from __future__ import annotations

import sys
from pathlib import Path

TOOLS_ROOT = Path(__file__).resolve().parents[1]
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from triwhirl_tool.geometry import (
    UPRIGHT_HALF_PERIOD_DEG,
    UPRIGHT_HALF_PERIOD_RAD,
    UPRIGHT_PERIOD_DEG,
    UPRIGHT_PERIOD_RAD,
    VERTEX_IDS,
    VertexMatch,
    angle_diff_deg,
    classify_vertex_deg,
    periodic_upright_error_deg,
    periodic_upright_error_rad,
    vertex_centers_deg,
    wrap_deg,
)

__all__ = [
    "VERTEX_IDS",
    "VertexMatch",
    "UPRIGHT_PERIOD_DEG",
    "UPRIGHT_HALF_PERIOD_DEG",
    "UPRIGHT_PERIOD_RAD",
    "UPRIGHT_HALF_PERIOD_RAD",
    "wrap_deg",
    "angle_diff_deg",
    "periodic_upright_error_deg",
    "periodic_upright_error_rad",
    "vertex_centers_deg",
    "classify_vertex_deg",
]
