from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from pathlib import Path
from typing import Sequence

from ..ble import DEVICE_NAME
from .log import (
    _close_line_transport,
    _open_line_transport,
    _parse_key_values,
    _wait_console,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run an untethered BLE-only bidirectional FOC direction check. "
            "Each polarity starts from rest and is verified from AS5600 wheel velocity."
        )
    )
    parser.add_argument(
        "--motor-config",
        type=Path,
        default=Path("artifacts/motor-config.json"),
        help="motor electrical calibration JSON",
    )
    parser.add_argument("--volts", type=float, default=0.5, help="absolute Vq test voltage")
    parser.add_argument(
        "--drive-seconds",
        type=float,
        default=1.0,
        help="time to drive each polarity before sampling velocity",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=10.0,
        help="maximum time to wait for the wheel to return near rest between polarities",
    )
    parser.add_argument(
        "--stop-threshold",
        type=float,
        default=0.10,
        help="absolute wheel speed considered stopped [rad/s]",
    )
    parser.add_argument(
        "--min-speed",
        type=float,
        default=0.50,
        help="minimum absolute driven wheel speed required for a decisive result [rad/s]",
    )
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser


def _load_motor_config(path: Path) -> tuple[int, int, float]:
    try:
        # utf-8-sig accepts both ordinary UTF-8 and Windows PowerShell UTF-8 BOM files.
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        pole_pairs = int(data["pole_pairs"])
        sensor_dir = int(data["sensor_dir"])
        offset_rad = float(data["offset_rad"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load motor config {path}: {exc}") from exc
    if not 1 <= pole_pairs <= 64 or sensor_dir not in (-1, 1) or not math.isfinite(offset_rad):
        raise RuntimeError(f"invalid motor config values in {path}")
    return pole_pairs, sensor_dir, offset_rad


async def _request_status(transport, timeout_s: float) -> tuple[str, dict[str, str]]:
    await transport.send("status")
    line = await _wait_console(transport, prefixes=("status,",), timeout_s=timeout_s)
    return line, _parse_key_values(line, "status")


def _float_field(values: dict[str, str], key: str) -> float:
    try:
        value = float(values[key])
    except (KeyError, ValueError) as exc:
        raise RuntimeError(f"firmware status missing/invalid {key}") from exc
    if not math.isfinite(value):
        raise RuntimeError(f"firmware status {key} is not finite")
    return value


async def _wait_for_rest(
    transport,
    *,
    timeout_s: float,
    threshold_rad_s: float,
    command_timeout_s: float,
) -> float:
    deadline = time.monotonic() + timeout_s
    last_speed = math.inf
    while time.monotonic() < deadline:
        _line, values = await _request_status(transport, command_timeout_s)
        last_speed = _float_field(values, "vel_rad_s")
        if abs(last_speed) <= threshold_rad_s:
            return last_speed
        await asyncio.sleep(0.20)
    raise RuntimeError(
        f"wheel did not settle below {threshold_rad_s:.3f} rad/s; last={last_speed:.6f} rad/s"
    )


async def _drive_and_sample(
    transport,
    *,
    vq_v: float,
    drive_seconds: float,
    timeout_s: float,
) -> tuple[str, float]:
    await transport.send(f"motor vq {vq_v:.9g}")
    print(
        await _wait_console(
            transport,
            prefixes=("OK motor FOC vq_v=",),
            timeout_s=timeout_s,
        )
    )
    await asyncio.sleep(drive_seconds)
    line, values = await _request_status(transport, timeout_s)
    speed = _float_field(values, "vel_rad_s")
    print(line)
    return line, speed


async def _run(args: argparse.Namespace) -> int:
    finite_positive = (
        args.volts,
        args.drive_seconds,
        args.settle_timeout,
        args.stop_threshold,
        args.min_speed,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in finite_positive):
        raise RuntimeError("volts/timing/threshold arguments must be finite and > 0")
    if args.volts > 3.0:
        raise RuntimeError("--volts must be <= 3.0 V")

    pole_pairs, sensor_dir, offset_rad = _load_motor_config(args.motor_config)
    client, transport = await _open_line_transport(args)
    try:
        await transport.send("motor stop")
        print(await _wait_console(transport, prefixes=("OK motor stop",), timeout_s=args.timeout))

        await transport.send("fault clear")
        print(
            await _wait_console(
                transport,
                prefixes=("OK fault clear", "OK fault already clear"),
                timeout_s=args.timeout,
            )
        )

        await transport.send(
            f"motor config {pole_pairs} {sensor_dir} {offset_rad:.9g}"
        )
        print(
            await _wait_console(
                transport,
                prefixes=("OK motor config",),
                timeout_s=args.timeout,
            )
        )

        print("waiting for wheel rest before +Vq test...")
        rest0 = await _wait_for_rest(
            transport,
            timeout_s=args.settle_timeout,
            threshold_rad_s=args.stop_threshold,
            command_timeout_s=args.timeout,
        )
        print(f"rest_before_positive={rest0:.6f} rad/s")

        _, positive_speed = await _drive_and_sample(
            transport,
            vq_v=args.volts,
            drive_seconds=args.drive_seconds,
            timeout_s=args.timeout,
        )
        await transport.send("motor stop")
        print(await _wait_console(transport, prefixes=("OK motor stop",), timeout_s=args.timeout))

        print("waiting for wheel rest before -Vq test...")
        rest1 = await _wait_for_rest(
            transport,
            timeout_s=args.settle_timeout,
            threshold_rad_s=args.stop_threshold,
            command_timeout_s=args.timeout,
        )
        print(f"rest_before_negative={rest1:.6f} rad/s")

        _, negative_speed = await _drive_and_sample(
            transport,
            vq_v=-args.volts,
            drive_seconds=args.drive_seconds,
            timeout_s=args.timeout,
        )
        await transport.send("motor stop")
        print(await _wait_console(transport, prefixes=("OK motor stop",), timeout_s=args.timeout))

        await transport.send("fault status")
        fault_line = await _wait_console(
            transport, prefixes=("fault,",), timeout_s=args.timeout
        )
        print(fault_line)
        fault = _parse_key_values(fault_line, "fault")

        print(
            "motor_direction_result,"
            f"vq_pos={args.volts:.6f},vel_pos={positive_speed:.6f},"
            f"vq_neg={-args.volts:.6f},vel_neg={negative_speed:.6f}"
        )

        if fault.get("latched") != "0":
            print("MOTOR_DIRECTION_CHECK_FAIL reason=safety_fault")
            return 2
        if abs(positive_speed) < args.min_speed or abs(negative_speed) < args.min_speed:
            print(
                "MOTOR_DIRECTION_CHECK_FAIL reason=insufficient_speed "
                f"min_required={args.min_speed:.3f}"
            )
            return 2
        if positive_speed * negative_speed >= 0.0:
            print("MOTOR_DIRECTION_CHECK_FAIL reason=same_velocity_sign")
            return 2

        print("MOTOR_DIRECTION_CHECK_PASS")
        return 0
    finally:
        if client.is_connected:
            try:
                await transport.send("motor stop")
                await asyncio.sleep(0.05)
            except Exception:
                pass
        await _close_line_transport(client, transport)


def motor_direction_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    try:
        return asyncio.run(_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
