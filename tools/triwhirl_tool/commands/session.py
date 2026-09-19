from __future__ import annotations

import argparse
import asyncio
import math
import time
from pathlib import Path
from typing import Sequence

from .log import (
    _close_line_transport,
    _open_line_transport,
    _request_log_status,
    _wait_console,
    decode_main,
    download_main,
)
from ..ble import DEVICE_NAME


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a complete firmware-owned TWLG recording lifecycle. The BLE "
            "control connection stays open from prepare through stop so the "
            "requested recording interval is not extended by reconnect latency."
        )
    )
    parser.add_argument(
        "seconds",
        nargs="?",
        type=float,
        default=45.0,
        help="requested recording interval [s]",
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="optionally decode the downloaded TWLG to this CSV path",
    )
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument(
        "--reserve-seconds",
        type=float,
        default=2.0,
        help="extra prepared capacity beyond the requested recording interval [s]",
    )
    parser.add_argument("--prepare-timeout", type=float, default=90.0)
    parser.add_argument("--control-timeout", type=float, default=5.0)
    parser.add_argument("--finalize-timeout", type=float, default=20.0)
    parser.add_argument("--download-timeout", type=float, default=60.0)
    return parser


def _ble_args(args: argparse.Namespace) -> list[str]:
    result = [
        "--name",
        str(args.name),
        "--scan-timeout",
        f"{args.scan_timeout:.9g}",
    ]
    if args.address:
        result.extend(("--address", str(args.address)))
    return result


async def _wait_ready(
    transport,
    *,
    timeout_s: float,
) -> None:
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
    raise RuntimeError(f"timed out after {timeout_s:.1f}s waiting for logger state=ready")


async def _wait_complete(
    transport,
    *,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        _line, status = await _request_log_status(transport, 3.0)
        state = status.get("state", "unknown")
        buffered = status.get("buffered_bytes", "?")
        if state == "complete":
            print(
                f"log state=complete records={status.get('records_written', '?')} "
                f"logical_bytes={status.get('logical_bytes', '?')} "
                f"dropped={status.get('dropped_records', '?')}"
            )
            return
        if state == "error":
            raise RuntimeError("logger entered state=error while stopping")
        print(f"log state={state} buffered_bytes={buffered}")
        await asyncio.sleep(0.1)
    raise RuntimeError("timed out waiting for logger state=complete")


async def _record_run(
    args: argparse.Namespace,
    prepared_seconds: float,
) -> bool:
    client, transport = await _open_line_transport(args)
    started = False
    interrupted = False
    try:
        await transport.send(f"log prepare {prepared_seconds:.9g}")
        prepare_line = await _wait_console(
            transport,
            prefixes=("OK log prepare",),
            timeout_s=args.control_timeout,
        )
        print(prepare_line)
        await _wait_ready(transport, timeout_s=args.prepare_timeout)

        await transport.send("log start")
        start_line = await _wait_console(
            transport,
            prefixes=("OK log start",),
            timeout_s=args.control_timeout,
        )
        print(start_line)
        started = True

        try:
            await asyncio.sleep(args.seconds)
        except asyncio.CancelledError:
            interrupted = True
            print("recording interrupted; finalizing the captured TWLG before exit")

        if started:
            await transport.send("log stop")
            stop_line = await _wait_console(
                transport,
                prefixes=("OK log stopping",),
                timeout_s=args.control_timeout,
            )
            print(stop_line)
            await _wait_complete(transport, timeout_s=args.finalize_timeout)
        return interrupted
    finally:
        await _close_line_transport(client, transport)


def session_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    if not math.isfinite(args.seconds) or args.seconds <= 0.0:
        print("error: seconds must be finite and > 0")
        return 2
    if not math.isfinite(args.reserve_seconds) or args.reserve_seconds < 0.0:
        print("error: --reserve-seconds must be finite and >= 0")
        return 2

    prepared_seconds = args.seconds + args.reserve_seconds
    ble = _ble_args(args)

    print(
        f"TWLG session: prepare={prepared_seconds:.3f}s, "
        f"record={args.seconds:.3f}s, output={args.output}"
    )

    try:
        interrupted = asyncio.run(_record_run(args, prepared_seconds))
    except KeyboardInterrupt:
        print("error: recording interrupted before firmware finalization completed")
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1

    rc = download_main(
        [
            "-o",
            str(args.output),
            *ble,
            "--timeout",
            f"{args.download_timeout:.9g}",
        ]
    )
    if rc != 0:
        return rc

    if args.csv is not None:
        rc = decode_main([str(args.output), "-o", str(args.csv)])
        if rc != 0:
            return rc

    return 130 if interrupted else 0
