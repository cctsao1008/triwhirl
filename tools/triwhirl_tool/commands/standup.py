from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from pathlib import Path
from typing import Sequence

from ..ble import DEVICE_NAME
from ..host_log import host_print as print
from ..host_log import print_session_header
from .log import (
    _close_line_transport,
    _normalize_console_line,
    _open_line_transport,
    _parse_key_values,
    _wait_console,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the firmware-owned TRC-V1.1 vendor-aligned autonomous "
            "swing-up -> balance controller over BLE."
        )
    )
    parser.add_argument(
        "--motor-config", type=Path, default=Path("artifacts/motor-config.json")
    )
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="bounded trial duration in seconds; 0 leaves standup active",
    )
    parser.add_argument("--poll-period", type=float, default=0.25)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser


def _load_motor_config(path: Path) -> tuple[int, int, float]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
        pole_pairs = int(payload["pole_pairs"])
        sensor_dir = int(payload["sensor_dir"])
        offset_rad = float(payload["offset_rad"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load motor config {path}: {exc}") from exc
    if not 1 <= pole_pairs <= 64:
        raise RuntimeError("motor pole_pairs must be in [1, 64]")
    if sensor_dir not in (-1, 1):
        raise RuntimeError("motor sensor_dir must be -1 or 1")
    if not math.isfinite(offset_rad):
        raise RuntimeError("motor offset_rad must be finite")
    return pole_pairs, sensor_dir, offset_rad


def _validate(args: argparse.Namespace) -> None:
    if not 50 <= args.imu_samples <= 5000:
        raise RuntimeError("--imu-samples must be in [50, 5000]")
    if not math.isfinite(args.duration) or args.duration < 0.0:
        raise RuntimeError("--duration must be finite and >= 0")
    if not math.isfinite(args.poll_period) or args.poll_period <= 0.0:
        raise RuntimeError("--poll-period must be finite and > 0")


async def _fault_status(transport, timeout: float) -> tuple[str, dict[str, str]]:
    await transport.send("fault status")
    line = await _wait_console(transport, prefixes=("fault,",), timeout_s=timeout)
    return line, _parse_key_values(line, "fault")


async def _balance_status(transport, timeout: float) -> tuple[str, dict[str, str]]:
    await transport.send("balance status")
    line = await _wait_console(transport, prefixes=("balance,",), timeout_s=timeout)
    return line, _parse_key_values(line, "balance")


async def _run(args: argparse.Namespace) -> int:
    pole_pairs, sensor_dir, offset_rad = _load_motor_config(args.motor_config)
    client, transport = await _open_line_transport(args)
    started = False
    try:
        await transport.send("motor stop")
        print(await _wait_console(transport, prefixes=("OK motor stop",), timeout_s=args.timeout))

        fault_line, fault = await _fault_status(transport, args.timeout)
        print(fault_line)
        if fault.get("latched") == "1":
            await transport.send("fault clear")
            print(
                await _wait_console(
                    transport,
                    prefixes=("OK fault clear", "OK fault already clear"),
                    timeout_s=args.timeout,
                )
            )

        await transport.send(f"motor config {pole_pairs} {sensor_dir} {offset_rad:.9g}")
        print(await _wait_console(transport, prefixes=("OK motor config",), timeout_s=args.timeout))

        print("keep the unit stationary in its normal resting orientation for gyro calibration")
        await transport.send(f"imu calibrate {args.imu_samples}")
        print(
            await _wait_console(
                transport,
                prefixes=("OK imu gyro calibration started",),
                timeout_s=args.timeout,
            )
        )
        print(
            await _wait_console(
                transport,
                prefixes=("OK imu gyro calibration bx=",),
                timeout_s=max(10.0, args.imu_samples * 0.004),
            )
        )

        await transport.send("attitude reset")
        print(await _wait_console(transport, prefixes=("OK attitude reset",), timeout_s=args.timeout))

        # After reset there is intentionally no synthesized balance config.
        # Firmware therefore selects the built-in TRC-V1.1 vendor standup path:
        # 0.42/0.168 V swing-up -> LQR wheel-velocity target -> velocity PI -> Vq.
        print("starting vendor-aligned autonomous standup (upright reference 68 deg)")
        await transport.send("balance start")
        print(
            await _wait_console(
                transport,
                prefixes=("OK balance start",),
                timeout_s=args.timeout,
            )
        )
        started = True

        if args.duration == 0.0:
            print("STANDUP_ACTIVE")
            started = False
            return 0

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            await asyncio.sleep(min(args.poll_period, max(0.0, deadline - time.monotonic())))
            line, status = await _balance_status(transport, args.timeout)
            print(_normalize_console_line(line))
            if status.get("active") != "1":
                fault_line, _fault = await _fault_status(transport, args.timeout)
                raise RuntimeError(f"standup became inactive: {line}; {fault_line}")

        await transport.send("balance stop")
        print(await _wait_console(transport, prefixes=("OK balance stop",), timeout_s=args.timeout))
        started = False
        print("STANDUP_TRIAL_COMPLETE")
        return 0
    finally:
        if started and client.is_connected:
            try:
                await transport.send("balance stop")
                await asyncio.sleep(0.1)
            except Exception:
                pass
        await _close_line_transport(client, transport)


def standup_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    print_session_header()
    try:
        _validate(args)
        return asyncio.run(_run(args))
    except KeyboardInterrupt:
        print("interrupted; standup stop requested")
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
