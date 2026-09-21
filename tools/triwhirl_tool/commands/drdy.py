from __future__ import annotations

import argparse
import asyncio
import math
from typing import Mapping, Sequence

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
            "Observe the candidate MPU6050 DATA_RDY GPIO without granting it "
            "control authority. The command measures edge-count deltas from "
            "two imu-status snapshots."
        )
    )
    parser.add_argument(
        "seconds",
        nargs="?",
        type=float,
        default=5.0,
        help="observation interval [s] (default: 5)",
    )
    parser.add_argument(
        "--require-1khz",
        action="store_true",
        help="return non-zero unless the observed edge rate is 800..1200 Hz",
    )
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser


def _int_field(values: Mapping[str, str], key: str) -> int:
    if key not in values:
        raise RuntimeError(f"firmware response missing {key}")
    try:
        return int(values[key], 0)
    except ValueError as exc:
        raise RuntimeError(f"invalid integer field {key}={values[key]!r}") from exc


async def _read_imu_status(transport, timeout_s: float) -> tuple[str, dict[str, str]]:
    await transport.send("imu status")
    line = await _wait_console(
        transport,
        prefixes=("imu,",),
        timeout_s=timeout_s,
    )
    return line, _parse_key_values(line, "imu")


def _delta(after: Mapping[str, str], before: Mapping[str, str], key: str) -> int:
    end = _int_field(after, key)
    start = _int_field(before, key)
    if end < start:
        raise RuntimeError(f"counter {key} moved backwards ({start} -> {end})")
    return end - start


def _classify(
    gpio: int,
    probe_only: int,
    edge_rate_hz: float,
    imu_ready: int,
) -> str:
    if gpio < 0:
        return "DISABLED"
    if 800.0 <= edge_rate_hz <= 1200.0:
        return "CANDIDATE_1KHZ" if probe_only else "VERIFIED_1KHZ"
    # DATA_RDY is enabled during successful MPU initialization. If initialization
    # is down, the chip may never have received INT_ENABLE, so zero/off-rate edges
    # cannot reject the physical routing hypothesis.
    if imu_ready == 0:
        return "INCONCLUSIVE_IMU_NOT_READY"
    if edge_rate_hz == 0.0:
        return "NO_EDGES" if probe_only else "VERIFIED_NO_EDGES"
    return "INCONCLUSIVE" if probe_only else "VERIFIED_UNEXPECTED_RATE"


async def _run(args: argparse.Namespace) -> int:
    if not math.isfinite(args.seconds) or args.seconds <= 0.0:
        raise RuntimeError("seconds must be finite and > 0")

    client, transport = await _open_line_transport(args)
    try:
        first_line, first = await _read_imu_status(transport, args.timeout)
        print(first_line)
        imu_ready = _int_field(first, "ready")
        if imu_ready == 0:
            print(
                "DRDY_PROBE_NOTE imu_ready=0; GPIO observation is passive only. "
                "No/off-rate edges are inconclusive because MPU INT_ENABLE may "
                "not have been configured. Balance remains blocked until MPU6050 "
                "I2C initialization succeeds."
            )

        gpio = _int_field(first, "drdy_gpio")
        probe_only = _int_field(first, "drdy_probe_only")
        if gpio < 0:
            print("DRDY_PROBE_DISABLED")
            return 2

        print(
            f"observing MPU6050 DRDY candidate gpio={gpio} "
            f"probe_only={probe_only} for {args.seconds:.3f} seconds..."
        )
        await asyncio.sleep(args.seconds)
        second_line, second = await _read_imu_status(transport, args.timeout)
        print(second_line)

        end_gpio = _int_field(second, "drdy_gpio")
        end_probe_only = _int_field(second, "drdy_probe_only")
        end_imu_ready = _int_field(second, "ready")
        if end_gpio != gpio or end_probe_only != probe_only:
            raise RuntimeError("DRDY routing state changed during observation")
        if end_imu_ready != imu_ready:
            raise RuntimeError("IMU readiness changed during observation")

        edges = _delta(second, first, "drdy_edges")
        consumed = _delta(second, first, "drdy_consumed")
        fallback = _delta(second, first, "drdy_fallback_reads")
        edge_rate_hz = edges / args.seconds
        state = _classify(gpio, probe_only, edge_rate_hz, imu_ready)
        print(
            "drdy_probe,"
            f"state={state},gpio={gpio},probe_only={probe_only},"
            f"seconds={args.seconds:.3f},edges={edges},"
            f"rate_hz={edge_rate_hz:.3f},consumed={consumed},fallback_reads={fallback}"
        )

        if state in {"CANDIDATE_1KHZ", "VERIFIED_1KHZ"}:
            print("DRDY_PROBE_MATCH")
            return 0
        if args.require_1khz:
            print("DRDY_PROBE_NO_MATCH")
            return 2
        print("DRDY_PROBE_OBSERVED")
        return 0
    finally:
        await _close_line_transport(client, transport)


def drdy_probe_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    try:
        return asyncio.run(_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
