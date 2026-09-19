#!/usr/bin/env python3
"""Acquire small-angle untethered body departures around the upright vertex.

This tool is intentionally different from body_free_ble.py: it does not record a
full fall. It waits until the user is holding the body steadily at the intended
vertex/upright posture, resets attitude from gravity, estimates that held angle
as theta_ref, then arms a release detector. After release it records only the
local departure until the body leaves the configured angular window.

Multiple trials are collected in one BLE session so the local model can be fit
from more than one departure without repeating setup or gyro calibration.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import statistics
import time
from collections import deque
from pathlib import Path

from bleak import BleakClient, BleakScanner

from acquire_ble import (
    DEVICE_NAME,
    SCHEMA_VERSION,
    TELEMETRY_FIELDS,
    TX_UUID,
    BleLineTransport,
    parse_telemetry,
    telemetry_fault_mask,
)
from body_free_ble import prepare_imu, read_until_prefix


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture small-angle upright departures over BLE."
    )
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument("--stable-window", type=float, default=0.8)
    parser.add_argument("--stable-rate", type=float, default=0.10,
                        help="maximum |theta_rate| during held-stable detection [rad/s]")
    parser.add_argument("--stable-angle-deg", type=float, default=1.0,
                        help="maximum peak-to-peak theta during held-stable detection [deg]")
    parser.add_argument("--trigger-rate", type=float, default=0.15,
                        help="release trigger |theta_rate| [rad/s]")
    parser.add_argument("--trigger-angle-deg", type=float, default=1.5,
                        help="release trigger |theta-theta_ref| [deg]")
    parser.add_argument("--max-angle-deg", type=float, default=8.0,
                        help="stop local capture at this |theta-theta_ref| [deg]")
    parser.add_argument("--max-local-duration", type=float, default=0.8,
                        help="maximum post-trigger local capture duration [s]")
    parser.add_argument("--hold-timeout", type=float, default=45.0)
    parser.add_argument("--release-timeout", type=float, default=20.0)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def angle_diff(angle: float, reference: float) -> float:
    return math.atan2(math.sin(angle - reference), math.cos(angle - reference))


def telemetry_dict(values: list[str]) -> dict[str, str]:
    return dict(zip(TELEMETRY_FIELDS, values))


def valid_state(row: dict[str, str]) -> bool:
    return (
        row.get("attitude_ok") == "1"
        and row.get("imu_ok") == "1"
        and row.get("vel_valid") == "1"
        and int(row.get("fault_mask", "0"), 0) == 0
    )


async def discover(args: argparse.Namespace):
    if args.address:
        return args.address
    print(f"scanning for BLE device {args.name!r} ...")
    device = await BleakScanner.find_device_by_name(args.name, timeout=args.scan_timeout)
    if device is None:
        raise RuntimeError(f"BLE device {args.name!r} not found")
    print(f"found {device.name or args.name}: {device.address}")
    return device


async def next_telemetry(transport: BleLineTransport, timeout_s: float = 1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = await transport.read_line(min(0.25, deadline - time.monotonic()))
        if not line:
            continue
        if line.startswith("FAULT,"):
            raise RuntimeError(line)
        values = parse_telemetry(line)
        if values is None:
            continue
        mask = telemetry_fault_mask(values)
        if mask != 0:
            raise RuntimeError(f"firmware fault_mask became nonzero: 0x{mask:08x}")
        row = telemetry_dict(values)
        if valid_state(row):
            return values, row
    return None, None


async def wait_for_stable_hold(
    transport: BleLineTransport,
    trial: int,
    window_s: float,
    max_rate: float,
    max_span_rad: float,
    timeout_s: float,
) -> tuple[float, list[tuple[list[str], dict[str, str]]]]:
    # Telemetry is nominally 50 Hz. Keep more than enough points and determine
    # the window from firmware timestamps rather than host arrival timing.
    history: deque[tuple[list[str], dict[str, str]]] = deque(maxlen=256)
    deadline = time.monotonic() + timeout_s
    last_message = 0.0

    while time.monotonic() < deadline:
        values, row = await next_telemetry(transport, 1.0)
        if row is None:
            continue
        history.append((values, row))
        now = time.monotonic()
        if now - last_message > 3.0:
            print(f"trial {trial}: hold the same vertex upright and keep it still ...")
            last_message = now

        newest_us = int(row["t_us"])
        selected = [
            item for item in history
            if (newest_us - int(item[1]["t_us"])) * 1.0e-6 <= window_s
        ]
        if len(selected) < 8:
            continue
        span_s = (int(selected[-1][1]["t_us"]) - int(selected[0][1]["t_us"])) * 1.0e-6
        if span_s < 0.85 * window_s:
            continue
        thetas = [float(item[1]["theta_rad"]) for item in selected]
        rates = [abs(float(item[1]["theta_rate_rad_s"])) for item in selected]
        if max(thetas) - min(thetas) <= max_span_rad and max(rates) <= max_rate:
            return statistics.median(thetas), selected

    raise RuntimeError(f"trial {trial}: upright hold did not become stable before timeout")


async def run(args: argparse.Namespace) -> int:
    if args.trials < 1:
        raise RuntimeError("--trials must be >= 1")
    if args.stable_window <= 0.0 or args.max_local_duration <= 0.0:
        raise RuntimeError("capture durations must be > 0")
    if args.max_angle_deg <= args.trigger_angle_deg:
        raise RuntimeError("--max-angle-deg must exceed --trigger-angle-deg")

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = output.with_suffix(output.suffix + ".json")
    target = await discover(args)
    resolved_address = getattr(target, "address", None) or args.address
    started_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    trial_metadata: list[dict[str, object]] = []
    total_rows = 0
    completed = False

    stable_span_rad = math.radians(args.stable_angle_deg)
    trigger_angle_rad = math.radians(args.trigger_angle_deg)
    max_angle_rad = math.radians(args.max_angle_deg)

    try:
        async with BleakClient(target) as client:
            if not client.is_connected:
                raise RuntimeError("BLE connection failed")
            transport = BleLineTransport(client)
            await client.start_notify(TX_UUID, transport.on_notify)
            await asyncio.sleep(0.2)

            try:
                await transport.send("motor stop")
                await transport.send("telemetry off")
                await prepare_imu(transport, args.imu_samples)

                with output.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow((
                        "schema_version", "trial", "phase", "theta_ref_rad",
                        *TELEMETRY_FIELDS,
                    ))
                    await transport.send("telemetry on")

                    for trial in range(1, args.trials + 1):
                        # First find a genuinely still held posture. Then reset
                        # attitude from gravity in that exact posture and repeat
                        # the stable-window acquisition so theta_ref belongs to
                        # the post-reset estimator state.
                        _, _ = await wait_for_stable_hold(
                            transport, trial, args.stable_window,
                            args.stable_rate, stable_span_rad, args.hold_timeout,
                        )
                        await transport.send("attitude reset")
                        await read_until_prefix(
                            transport, "OK attitude reset from accelerometer", 3.0
                        )
                        theta_ref, baseline = await wait_for_stable_hold(
                            transport, trial, args.stable_window,
                            args.stable_rate, stable_span_rad, args.hold_timeout,
                        )

                        theta_ref_deg = math.degrees(theta_ref)
                        print(
                            f"trial {trial}: theta_ref={theta_ref:.6f} rad "
                            f"({theta_ref_deg:.2f} deg)"
                        )
                        if abs(theta_ref - 1.169) > math.radians(15.0):
                            print(
                                "warning: held angle is far from the previously observed "
                                "upright vertex (~67 deg); check that the same vertex is down"
                            )

                        for values, _row in baseline:
                            writer.writerow((SCHEMA_VERSION, trial, "hold", theta_ref, *values))
                            total_rows += 1

                        print(
                            f"trial {trial} ARMED: release your fingers without pushing; "
                            f"local capture stops at ±{args.max_angle_deg:.1f} deg"
                        )

                        release_deadline = time.monotonic() + args.release_timeout
                        consecutive = 0
                        trigger_t_us: int | None = None
                        trigger_reason: str | None = None
                        local_rows = 0
                        max_deviation = 0.0
                        end_reason = "release_timeout"

                        while time.monotonic() < release_deadline:
                            values, row = await next_telemetry(transport, 1.0)
                            if row is None:
                                continue
                            theta = float(row["theta_rad"])
                            rate = float(row["theta_rate_rad_s"])
                            deviation = abs(angle_diff(theta, theta_ref))
                            max_deviation = max(max_deviation, deviation)

                            if trigger_t_us is None:
                                writer.writerow((
                                    SCHEMA_VERSION, trial, "armed", theta_ref, *values
                                ))
                                total_rows += 1
                                rate_hit = abs(rate) >= args.trigger_rate
                                angle_hit = deviation >= trigger_angle_rad
                                if rate_hit or angle_hit:
                                    consecutive += 1
                                else:
                                    consecutive = 0
                                if consecutive >= 2:
                                    trigger_t_us = int(row["t_us"])
                                    trigger_reason = "rate" if rate_hit else "angle"
                                    print(
                                        f"trial {trial} RELEASE detected: "
                                        f"{trigger_reason}, theta_error="
                                        f"{math.degrees(angle_diff(theta, theta_ref)):.2f} deg, "
                                        f"rate={rate:.3f} rad/s"
                                    )
                                continue

                            writer.writerow((
                                SCHEMA_VERSION, trial, "local", theta_ref, *values
                            ))
                            total_rows += 1
                            local_rows += 1
                            elapsed = (int(row["t_us"]) - trigger_t_us) * 1.0e-6
                            if deviation >= max_angle_rad:
                                end_reason = "angle_limit"
                                break
                            if elapsed >= args.max_local_duration:
                                end_reason = "duration_limit"
                                break

                        if trigger_t_us is None:
                            raise RuntimeError(
                                f"trial {trial}: no release detected before timeout"
                            )

                        trial_metadata.append({
                            "trial": trial,
                            "theta_ref_rad": theta_ref,
                            "theta_ref_deg": theta_ref_deg,
                            "trigger_t_us": trigger_t_us,
                            "trigger_reason": trigger_reason,
                            "local_rows": local_rows,
                            "max_abs_theta_error_deg": math.degrees(max_deviation),
                            "end_reason": end_reason,
                        })
                        print(
                            f"trial {trial} captured: {local_rows} local rows, "
                            f"end={end_reason}. Reposition and hold for the next trial."
                        )

                    completed = True
            finally:
                if client.is_connected:
                    for command in ("motor stop", "telemetry off"):
                        try:
                            await transport.send(command)
                        except Exception:
                            pass
                    try:
                        await client.stop_notify(TX_UUID)
                    except Exception:
                        pass
    finally:
        metadata = {
            "format": "triwhirl-body-local-run-v1",
            "telemetry_schema_version": SCHEMA_VERSION,
            "transport": "ble",
            "device_name": args.name,
            "device_address": resolved_address,
            "started": started_wall,
            "completed": completed,
            "imu_calibration_samples": max(50, min(5000, args.imu_samples)),
            "stable_window_s": args.stable_window,
            "stable_rate_rad_s": args.stable_rate,
            "stable_angle_deg": args.stable_angle_deg,
            "trigger_rate_rad_s": args.trigger_rate,
            "trigger_angle_deg": args.trigger_angle_deg,
            "max_angle_deg": args.max_angle_deg,
            "max_local_duration_s": args.max_local_duration,
            "total_rows": total_rows,
            "trials": trial_metadata,
            "csv": str(output),
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    print(f"saved {total_rows} rows -> {output}")
    print(f"saved run metadata -> {metadata_path}")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(run(args))
    except (RuntimeError, OSError, json.JSONDecodeError) as exc:
        print(f"error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
