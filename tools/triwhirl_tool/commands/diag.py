from __future__ import annotations

import argparse
import asyncio
import math
from typing import Sequence

from ..ble import DEVICE_NAME
from .log import _close_line_transport, _open_line_transport, _wait_console


def _add_ble_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=3.0)


def _simple_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    _add_ble_args(parser)
    return parser


def _timing_test_parser() -> argparse.ArgumentParser:
    parser = _simple_parser(
        "Reset firmware timing counters, measure for a fixed interval, then report them over BLE"
    )
    parser.add_argument(
        "seconds",
        nargs="?",
        type=float,
        default=5.0,
        help="measurement interval [s]",
    )
    return parser


async def _single_console_command(args: argparse.Namespace, command: str, prefix: str) -> int:
    client, transport = await _open_line_transport(args)
    try:
        await transport.send(command)
        line = await _wait_console(
            transport,
            prefixes=(prefix,),
            timeout_s=args.timeout,
        )
        print(line)
        return 0
    finally:
        await _close_line_transport(client, transport)


def timing_main(argv: Sequence[str]) -> int:
    args = _simple_parser("Read firmware realtime timing counters over BLE").parse_args(
        list(argv)
    )
    try:
        return asyncio.run(_single_console_command(args, "timing status", "timing,"))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


def timing_reset_main(argv: Sequence[str]) -> int:
    args = _simple_parser("Reset firmware realtime timing counters over BLE").parse_args(
        list(argv)
    )
    try:
        return asyncio.run(
            _single_console_command(args, "timing reset", "OK timing reset")
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


async def _timing_test_run(args: argparse.Namespace) -> int:
    if not math.isfinite(args.seconds) or args.seconds <= 0.0:
        raise RuntimeError("seconds must be finite and > 0")

    client, transport = await _open_line_transport(args)
    try:
        await transport.send("timing reset")
        reset_line = await _wait_console(
            transport,
            prefixes=("OK timing reset",),
            timeout_s=args.timeout,
        )
        print(reset_line)
        print(f"measuring for {args.seconds:.3f} seconds...")
        await asyncio.sleep(args.seconds)
        await transport.send("timing status")
        timing_line = await _wait_console(
            transport,
            prefixes=("timing,",),
            timeout_s=args.timeout,
        )
        print(timing_line)
        return 0
    finally:
        await _close_line_transport(client, transport)


def timing_test_main(argv: Sequence[str]) -> int:
    args = _timing_test_parser().parse_args(list(argv))
    try:
        return asyncio.run(_timing_test_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


async def _imu_run(args: argparse.Namespace) -> int:
    client, transport = await _open_line_transport(args)
    try:
        await transport.send("imu status")
        imu_line = await _wait_console(
            transport,
            prefixes=("imu,",),
            timeout_s=args.timeout,
        )
        print(imu_line)

        await transport.send("attitude status")
        attitude_line = await _wait_console(
            transport,
            prefixes=("attitude,",),
            timeout_s=args.timeout,
        )
        print(attitude_line)
        return 0
    finally:
        await _close_line_transport(client, transport)


def imu_main(argv: Sequence[str]) -> int:
    args = _simple_parser("Read IMU and attitude-estimator health over BLE").parse_args(
        list(argv)
    )
    try:
        return asyncio.run(_imu_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
