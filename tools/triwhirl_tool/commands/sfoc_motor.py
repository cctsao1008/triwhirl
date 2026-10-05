from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time
from typing import Sequence

from ..ble import DEVICE_NAME, TX_UUID, BleLineTransport, discover_target


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="twtool diag sfoc-motor",
        description=(
            "Run one bounded BLE-GATT SimpleFOC motor commissioning cycle: "
            "initFOC, +target velocity, stop, -target velocity, stop."
        ),
    )
    parser.add_argument("--pole-pairs", type=int, default=7)
    parser.add_argument("--supply-v", type=float, default=8.3)
    parser.add_argument("--limit-v", type=float, default=0.5)
    parser.add_argument("--align-v", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=5.0, help="absolute target velocity [rad/s]")
    parser.add_argument("--p", type=float, default=0.1, help="provisional velocity P gain")
    parser.add_argument("--tf", type=float, default=0.02, help="provisional SimpleFOC velocity LPF Tf [s]")
    parser.add_argument("--drive-seconds", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    return parser


def _parse_result(line: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for token in line.strip().split(",")[1:]:
        key, sep, value = token.partition("=")
        if sep:
            values[key] = value
    return values


def _float(values: dict[str, str], key: str) -> float:
    value = float(values[key])
    if not math.isfinite(value):
        raise ValueError(f"{key} is not finite")
    return value


async def _wait_line(
    transport: BleLineTransport,
    *,
    prefixes: tuple[str, ...],
    timeout_s: float,
) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = max(0.01, deadline - time.monotonic())
        line = await transport.read_line(min(remaining, 0.5))
        if line is None:
            continue
        if line.startswith(prefixes):
            return line
    raise RuntimeError(f"timeout waiting for {prefixes}")


async def _run(args: argparse.Namespace) -> int:
    from bleak import BleakClient

    finite_positive = (
        args.supply_v,
        args.limit_v,
        args.align_v,
        abs(args.speed),
        args.tf,
        args.drive_seconds,
        args.timeout,
        args.scan_timeout,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in finite_positive):
        raise RuntimeError("voltage/speed/timing/Tf arguments must be finite and > 0")
    if not math.isfinite(args.p) or args.p < 0.0:
        raise RuntimeError("--p must be finite and >= 0")

    duration_ms = int(round(args.drive_seconds * 1000.0))
    command = (
        f"commission {args.pole_pairs} {args.supply_v:.9g} {args.limit_v:.9g} "
        f"{args.align_v:.9g} {abs(args.speed):.9g} {args.p:.9g} {args.tf:.9g} "
        f"{duration_ms}"
    )

    target = await discover_target(
        name=args.name,
        address=args.address,
        scan_timeout=args.scan_timeout,
    )
    client = BleakClient(target)
    transport: BleLineTransport | None = None
    sent = False
    try:
        await client.connect()
        if not client.is_connected:
            raise RuntimeError("BLE connection failed")

        transport = BleLineTransport(client)
        await client.start_notify(TX_UUID, transport.on_notify)
        await asyncio.sleep(0.2)
        transport.drain()

        await transport.send("status")
        status_line = await _wait_line(
            transport,
            prefixes=("sfoc_motor_status,",),
            timeout_s=min(args.timeout, 5.0),
        )
        print(status_line)
        print("BLE GATT command path proven before motor init")

        await transport.send(command)
        sent = True
        print(command)

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            line = await transport.read_line(min(remaining, 0.5))
            if line is None:
                continue
            print(line)
            if line.startswith("ERR sfoc_motor"):
                return 2
            if not line.startswith("sfoc_motor_result,"):
                continue

            values = _parse_result(line)
            try:
                init_ok = values.get("init_ok") == "1"
                begin_stage = values.get("begin_stage", "unknown")
                aborted = values.get("aborted") == "1"
                faulted = values.get("backend_faulted") == "1"
                sensor_valid = values.get("sensor_valid") == "1"
                pos = _float(values, "pos_mean_rad_s")
                neg = _float(values, "neg_mean_rad_s")
            except (KeyError, ValueError) as exc:
                print(f"SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=malformed_result detail={exc}")
                return 2

            if not init_ok:
                print(
                    "SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=backend_begin "
                    f"stage={begin_stage}"
                )
                return 2
            if aborted:
                print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=aborted")
                return 2
            if faulted or not sensor_valid:
                print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=backend_or_sensor_fault")
                return 2
            if pos <= 0.0 or neg >= 0.0:
                print(
                    "SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=direction "
                    f"pos_mean={pos:.6f} neg_mean={neg:.6f}"
                )
                return 2

            print(
                "SIMPLEFOC_MOTOR_COMMISSION_PASS "
                f"pos_mean={pos:.6f}rad/s neg_mean={neg:.6f}rad/s"
            )
            return 0

        print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=result_timeout")
        return 2
    finally:
        if transport is not None and client.is_connected:
            try:
                await transport.send("stop")
                await asyncio.sleep(0.05)
            except Exception:
                pass
            try:
                await client.stop_notify(TX_UUID)
            except Exception:
                pass
        if client.is_connected:
            try:
                await client.disconnect()
            except Exception:
                pass


def sfoc_motor_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    try:
        return asyncio.run(_run(args))
    except ImportError:
        print("twtool: sfoc-motor requires bleak (python -m pip install bleak)", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("SIMPLEFOC_MOTOR_COMMISSION_ABORTED")
        return 130
    except RuntimeError as exc:
        print(f"SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=ble_transport detail={exc}")
        return 2
