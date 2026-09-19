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
from ..swing_log import write_fit_csv
from .download import download_main
from .log import (
    _close_line_transport,
    _normalize_console_line,
    _open_line_transport,
    _parse_key_values,
    _request_log_status,
    _wait_console,
    decode_main,
    inspect_main,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run firmware-owned swing identification. ESP32 owns all realtime "
            "pump/probe decisions; the host only supervises events and downloads TWLG."
        )
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, default=None, help="optional full decoded TWLG CSV")
    parser.add_argument(
        "--fit-csv",
        type=Path,
        default=None,
        help="fit-ready probe-window CSV; default: <output-stem>-active.csv",
    )
    parser.add_argument("--motor-config", type=Path, default=Path("artifacts/motor-config.json"))
    parser.add_argument("--captures", type=int, default=12)
    parser.add_argument("--pump-v-low", type=float, default=0.71)
    parser.add_argument("--pump-v-high", type=float, default=0.98)
    parser.add_argument("--capture-deg", type=float, default=8.0)
    parser.add_argument("--probe-exit-deg", type=float, default=12.0)
    parser.add_argument("--rearm-deg", type=float, default=18.0)
    parser.add_argument("--probe-duration", type=float, default=0.160)
    parser.add_argument("--rate-switch", type=float, default=0.03)
    parser.add_argument("--pump-polarity", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--vertex-a-deg", type=float, default=68.0)
    parser.add_argument("--max-duration", type=float, default=50.0)
    parser.add_argument("--reserve-seconds", type=float, default=2.0)
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--prepare-timeout", type=float, default=90.0)
    parser.add_argument("--finalize-timeout", type=float, default=20.0)
    parser.add_argument("--download-timeout", type=float, default=180.0)
    return parser


def _validate(args: argparse.Namespace) -> None:
    finite = (
        args.pump_v_low,
        args.pump_v_high,
        args.capture_deg,
        args.probe_exit_deg,
        args.rearm_deg,
        args.probe_duration,
        args.rate_switch,
        args.vertex_a_deg,
        args.max_duration,
        args.reserve_seconds,
    )
    if any(not math.isfinite(value) for value in finite):
        raise RuntimeError("swing-ID numeric arguments must be finite")
    if args.captures < 1:
        raise RuntimeError("--captures must be >= 1")
    if not (0.0 < args.pump_v_low <= args.pump_v_high <= 1.5):
        raise RuntimeError("require 0 < pump-v-low <= pump-v-high <= 1.5 V")
    if not (0.0 < args.capture_deg < args.probe_exit_deg < args.rearm_deg < 60.0):
        raise RuntimeError("require 0 < capture-deg < probe-exit-deg < rearm-deg < 60")
    if args.probe_duration <= 0.0 or args.max_duration <= 0.0:
        raise RuntimeError("probe/max durations must be > 0")
    if args.rate_switch < 0.0:
        raise RuntimeError("--rate-switch must be >= 0")
    if args.reserve_seconds < 0.0:
        raise RuntimeError("--reserve-seconds must be >= 0")
    if not 50 <= args.imu_samples <= 5000:
        raise RuntimeError("--imu-samples must be in [50, 5000]")


def _load_motor_config(path: Path) -> tuple[int, int, float]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        pole_pairs = int(data["pole_pairs"])
        sensor_dir = int(data["sensor_dir"])
        offset_rad = float(data["offset_rad"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load motor config {path}: {exc}") from exc
    if not 1 <= pole_pairs <= 64 or sensor_dir not in (-1, 1) or not math.isfinite(offset_rad):
        raise RuntimeError(f"invalid motor config values in {path}")
    return pole_pairs, sensor_dir, offset_rad


def _ble_args(args: argparse.Namespace) -> list[str]:
    result = ["--name", args.name, "--scan-timeout", str(args.scan_timeout)]
    if args.address:
        result.extend(("--address", args.address))
    return result


async def _wait_imu_calibration(transport, samples: int) -> None:
    await transport.send(f"imu calibrate {samples}")
    started = await _wait_console(
        transport,
        prefixes=("OK imu gyro calibration started",),
        timeout_s=5.0,
    )
    print(started)
    completed = await _wait_console(
        transport,
        prefixes=("OK imu gyro calibration bx=",),
        timeout_s=max(10.0, samples * 0.004),
    )
    print(completed)


async def _prepare_logger(transport, seconds: float, timeout_s: float) -> None:
    await transport.send(f"log prepare {seconds:.9g}")
    line = await _wait_console(
        transport,
        prefixes=("OK log prepare",),
        timeout_s=5.0,
    )
    print(line)
    deadline = time.monotonic() + timeout_s
    last_state = ""
    while time.monotonic() < deadline:
        _line, status = await _request_log_status(transport, 3.0)
        state = status.get("state", "unknown")
        if state != last_state:
            print(
                f"log state={state} prepared_bytes={status.get('prepared_bytes', '?')} "
                f"partition_bytes={status.get('partition_bytes', '?')}"
            )
            last_state = state
        if state == "ready":
            return
        if state in {"error", "unavailable"}:
            raise RuntimeError(f"logger entered state={state}")
        await asyncio.sleep(0.25)
    raise RuntimeError("timed out waiting for logger state=ready")


def _swing_config_command(args: argparse.Namespace) -> str:
    probe_ms = args.probe_duration * 1000.0
    return (
        "swing config "
        f"{args.captures} {args.pump_v_low:.9g} {args.pump_v_high:.9g} "
        f"{args.capture_deg:.9g} {args.probe_exit_deg:.9g} {args.rearm_deg:.9g} "
        f"{probe_ms:.9g} {args.rate_switch:.9g} {args.pump_polarity} "
        f"{args.vertex_a_deg:.9g} {args.max_duration:.9g}"
    )


def _parse_state_line(line: str) -> tuple[str, str]:
    normalized = _normalize_console_line(line)
    if normalized.startswith("event,swing_id,"):
        values = _parse_key_values(normalized, "event")
        return values.get("state", ""), values.get("reason", "")
    if normalized.startswith("swing,"):
        values = _parse_key_values(normalized, "swing")
        return values.get("state", ""), values.get("reason", "")
    return "", ""


async def _wait_for_terminal(transport, max_duration: float) -> tuple[str, str]:
    deadline = time.monotonic() + max_duration + 10.0
    next_poll = time.monotonic() + 2.0
    last_status_state = ""
    while time.monotonic() < deadline:
        remaining = max(0.05, min(0.5, deadline - time.monotonic()))
        line = await transport.read_line(remaining)
        if line:
            normalized = _normalize_console_line(line)
            if normalized.startswith("event,swing_id,"):
                print(normalized)
                state, reason = _parse_state_line(normalized)
                if state in {"complete", "aborted"}:
                    return state, reason
            elif normalized.startswith("FAULT,"):
                print(normalized)
            elif normalized.startswith("ERR "):
                raise RuntimeError(normalized)

        now = time.monotonic()
        if now >= next_poll:
            await transport.send("swing status")
            status_line = await _wait_console(
                transport,
                prefixes=("swing,",),
                timeout_s=2.0,
            )
            state, reason = _parse_state_line(status_line)
            if state != last_status_state:
                print(status_line)
                last_status_state = state
            if state in {"complete", "aborted"}:
                return state, reason
            next_poll = now + 2.0

    await transport.send("swing abort")
    try:
        await _wait_console(transport, prefixes=("OK swing abort",), timeout_s=2.0)
    except RuntimeError:
        pass
    raise RuntimeError("host supervision timeout; swing abort requested")


async def _wait_log_complete(transport, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_state = ""
    while time.monotonic() < deadline:
        _line, status = await _request_log_status(transport, 3.0)
        state = status.get("state", "unknown")
        if state != last_state:
            print(
                f"log state={state} records={status.get('records_written', '?')} "
                f"buffered={status.get('buffered_bytes', '?')} "
                f"dropped={status.get('dropped_records', '?')}"
            )
            last_state = state
        if state == "complete":
            return
        if state in {"error", "unavailable"}:
            raise RuntimeError(f"logger entered state={state}")
        await asyncio.sleep(0.1)
    raise RuntimeError("timed out waiting for TWLG finalization")


async def _run(args: argparse.Namespace) -> tuple[str, str]:
    pole_pairs, sensor_dir, offset_rad = _load_motor_config(args.motor_config)
    client, transport = await _open_line_transport(args)
    try:
        await transport.send("motor stop")
        print(
            await _wait_console(
                transport,
                prefixes=("OK motor stop",),
                timeout_s=3.0,
            )
        )

        await transport.send(
            f"motor config {pole_pairs} {sensor_dir} {offset_rad:.9g}"
        )
        print(
            await _wait_console(
                transport,
                prefixes=("OK motor config",),
                timeout_s=3.0,
            )
        )

        print("keep the unit stationary in its intended rocking orientation for gyro calibration")
        await _wait_imu_calibration(transport, args.imu_samples)

        await transport.send("attitude reset")
        print(
            await _wait_console(
                transport,
                prefixes=("OK attitude reset",),
                timeout_s=3.0,
            )
        )

        prepared_seconds = args.max_duration + args.reserve_seconds
        print(
            f"swing ID: prepare={prepared_seconds:.3f}s max_run={args.max_duration:.3f}s "
            f"captures={args.captures}"
        )
        await _prepare_logger(transport, prepared_seconds, args.prepare_timeout)

        await transport.send("log start")
        print(
            await _wait_console(
                transport,
                prefixes=("OK log start",),
                timeout_s=3.0,
            )
        )

        await transport.send(_swing_config_command(args))
        print(
            await _wait_console(
                transport,
                prefixes=("OK swing config",),
                timeout_s=3.0,
            )
        )
        await asyncio.sleep(0.05)
        transport.drain()

        await transport.send("swing start")
        print(
            await _wait_console(
                transport,
                prefixes=("OK swing start",),
                timeout_s=3.0,
            )
        )
        state, reason = await _wait_for_terminal(transport, args.max_duration)
        await _wait_log_complete(transport, args.finalize_timeout)
        return state, reason
    except KeyboardInterrupt:
        if client.is_connected:
            await transport.send("swing abort")
            await asyncio.sleep(0.2)
        raise
    finally:
        await _close_line_transport(client, transport)


def swing_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    print_session_header()
    try:
        _validate(args)
        state, reason = asyncio.run(_run(args))
    except KeyboardInterrupt:
        print("interrupted; abort requested")
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rc = download_main(
        [
            *_ble_args(args),
            "--timeout",
            str(args.download_timeout),
            "-o",
            str(args.output),
        ],
        session_header=False,
    )
    if rc != 0:
        return rc

    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        rc = decode_main([str(args.output), "-o", str(args.csv)])
        if rc != 0:
            return rc

    fit_csv = args.fit_csv
    if fit_csv is None:
        fit_csv = args.output.with_name(args.output.stem + "-active.csv")
    try:
        write_fit_csv(args.output, fit_csv, vertex_a_deg=args.vertex_a_deg)
    except RuntimeError as exc:
        if state == "complete":
            print(f"error: completed swing run has no usable probe windows: {exc}")
            return 1
        print(f"warning: no fit-ready probe CSV generated: {exc}")

    inspect_main([str(args.output)])
    if state == "complete":
        print("SWING_ID_COMPLETE")
        return 0
    print(f"SWING_ID_ABORTED reason={reason or 'unknown'}")
    return 3
