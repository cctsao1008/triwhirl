#!/usr/bin/env python3
"""Render a firmware balance command from a validated H-infinity synthesis JSON.

The synthesis artifact uses the project convention

    Vq = -K_inf * [theta_error, theta_rate, wheel_rate]^T

which is identical to the realtime BalanceController convention. This tool only
translates a stable synthesis artifact into the supervisor CLI command; it does
not flash firmware or start the motor unless the caller explicitly requests the
additional `balance start` line.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


FORMAT = "triwhirl-hinf-state-feedback-v1"


def positive_finite(value: float, name: str) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and > 0")
    return value


def load_command_values(
    path: Path,
    *,
    theta_reference_deg: float,
    capture_deg: float,
    fall_deg: float,
    vq_limit_v: float | None,
    wheel_limit_rad_s: float | None,
) -> tuple[float, float, float, float, float, float, float, float]:
    payload: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("format") != FORMAT:
        raise ValueError(f"unsupported synthesis format: {payload.get('format')!r}")
    if payload.get("all_vertices_stable") is not True:
        raise ValueError("refusing deployment command: not all plant vertices are stable")

    gain = payload.get("K_inf")
    if not isinstance(gain, list) or len(gain) != 3:
        raise ValueError("K_inf must contain exactly three gains")
    k_theta, k_rate, k_wheel = (float(value) for value in gain)
    if not all(math.isfinite(value) for value in (k_theta, k_rate, k_wheel)):
        raise ValueError("K_inf contains a non-finite value")

    theta_reference_deg = float(theta_reference_deg)
    if not math.isfinite(theta_reference_deg):
        raise ValueError("theta reference must be finite")
    capture_deg = positive_finite(float(capture_deg), "capture angle")
    fall_deg = positive_finite(float(fall_deg), "fall angle")
    if capture_deg >= fall_deg:
        raise ValueError("capture angle must be smaller than fall angle")
    if fall_deg > 60.0:
        raise ValueError("fall angle must be <= 60 degrees for the 120-degree upright coordinate")

    scales = payload.get("performance_scales")
    if not isinstance(scales, dict):
        scales = {}
    if vq_limit_v is None:
        if "Vq_v" not in scales:
            raise ValueError("--vq-limit-v is required when synthesis has no performance_scales.Vq_v")
        vq_limit_v = float(scales["Vq_v"])
    if wheel_limit_rad_s is None:
        if "wheel_rate_rad_s" not in scales:
            raise ValueError(
                "--wheel-limit-rad-s is required when synthesis has no performance_scales.wheel_rate_rad_s"
            )
        wheel_limit_rad_s = float(scales["wheel_rate_rad_s"])

    vq_limit_v = positive_finite(float(vq_limit_v), "Vq limit")
    wheel_limit_rad_s = positive_finite(
        float(wheel_limit_rad_s), "wheel-rate limit"
    )
    return (
        k_theta,
        k_rate,
        k_wheel,
        theta_reference_deg,
        capture_deg,
        fall_deg,
        vq_limit_v,
        wheel_limit_rad_s,
    )


def render_balance_command(values: tuple[float, ...]) -> str:
    if len(values) != 8:
        raise ValueError("balance command requires eight values")
    return "balance config " + " ".join(f"{value:.9g}" for value in values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a TriWhirl balance config command from H-infinity synthesis."
    )
    parser.add_argument("input", type=Path, help="*-hinf.json synthesis artifact")
    parser.add_argument(
        "--theta-reference-deg",
        type=float,
        required=True,
        help="measured physical upright reference for vertex A; B/C share the same 120-degree coordinate",
    )
    parser.add_argument("--capture-deg", type=float, default=6.0)
    parser.add_argument("--fall-deg", type=float, default=24.0)
    parser.add_argument("--vq-limit-v", type=float)
    parser.add_argument("--wheel-limit-rad-s", type=float)
    parser.add_argument(
        "--include-start",
        action="store_true",
        help="also print `balance start`; use only during deliberate near-upright hardware bring-up",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    values = load_command_values(
        args.input,
        theta_reference_deg=args.theta_reference_deg,
        capture_deg=args.capture_deg,
        fall_deg=args.fall_deg,
        vq_limit_v=args.vq_limit_v,
        wheel_limit_rad_s=args.wheel_limit_rad_s,
    )
    print(render_balance_command(values))
    if args.include_start:
        print("balance start")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
