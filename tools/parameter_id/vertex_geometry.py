#!/usr/bin/env python3
"""Compatibility wrapper for shared TriWhirl vertex geometry helpers."""

from __future__ import annotations

import sys
from pathlib import Path

TOOLS_ROOT = Path(__file__).resolve().parents[1]
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from triwhirl_tool.geometry import (
    VERTEX_IDS,
    VertexMatch,
    angle_diff_deg,
    classify_vertex_deg,
    vertex_centers_deg,
    wrap_deg,
)

__all__ = [
    "VERTEX_IDS",
    "VertexMatch",
    "wrap_deg",
    "angle_diff_deg",
    "vertex_centers_deg",
    "classify_vertex_deg",
]
