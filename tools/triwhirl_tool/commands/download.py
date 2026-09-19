from __future__ import annotations

import argparse
import asyncio
import re
import time
from pathlib import Path
from typing import Sequence

from .. import twlog
from ..ble import DEVICE_NAME, TX_UUID, drain_queue, discover_target, send_command
from ..host_log import host_print, print_session_header

MARKER = re.compile(
    rb"logdump,format=TWLG1,bytes=(\d+),record_size=(\d+),records=(\d+)\r?\n"
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Download a completed TWLG log over BLE")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="maximum BLE dump transfer time [s] (default: 180)",
    )
    return parser


def _progress_line(received: int, expected: int, elapsed_s: float) -> str:
    percent = 100.0 * received / expected if expected > 0 else 0.0
    rate_bps = received / elapsed_s if elapsed_s > 0.0 else 0.0
    remaining = max(0, expected - received)
    eta_s = remaining / rate_bps if rate_bps > 0.0 else float("inf")
    eta = f"{eta_s:.0f}s" if eta_s != float("inf") else "?"
    return (
        f"download {percent:5.1f}%  {received / 1024.0:.0f}/{expected / 1024.0:.0f} KiB  "
        f"{rate_bps / 1024.0:.1f} KiB/s  ETA {eta}"
    )


async def _download_run(args: argparse.Namespace) -> int:
    from bleak import BleakClient

    if args.timeout <= 0.0:
        raise RuntimeError("--timeout must be > 0")

    target = await discover_target(
        name=args.name,
        address=args.address,
        scan_timeout=args.scan_timeout,
    )
    queue: asyncio.Queue[bytes] = asyncio.Queue()

    def on_notify(_sender, data: bytearray) -> None:
        queue.put_nowait(bytes(data))

    transfer_started: float | None = None
    transfer_finished: float | None = None

    async with BleakClient(target) as client:
        if not client.is_connected:
            raise RuntimeError("BLE connection failed")
        await client.start_notify(TX_UUID, on_notify)
        await asyncio.sleep(0.2)
        await send_command(client, "telemetry off")
        await asyncio.sleep(0.05)
        drain_queue(queue)

        await send_command(client, "log dump")
        loop = asyncio.get_running_loop()
        deadline = loop.time() + args.timeout
        prefix = bytearray()
        payload = bytearray()
        expected_bytes: int | None = None
        record_size = 0
        record_count = 0
        last_progress = loop.time()

        while loop.time() < deadline:
            remaining = max(0.05, deadline - loop.time())
            try:
                chunk = await asyncio.wait_for(queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                break

            if expected_bytes is None:
                prefix.extend(chunk)
                match = MARKER.search(prefix)
                if match is None:
                    if len(prefix) > 8192:
                        del prefix[:-4096]
                    continue
                expected_bytes = int(match.group(1))
                record_size = int(match.group(2))
                record_count = int(match.group(3))
                if expected_bytes <= 0:
                    raise RuntimeError("firmware announced an empty log dump")
                if record_size != twlog.RECORD_BYTES:
                    raise RuntimeError(f"unexpected TWLG record size {record_size}")
                payload.extend(prefix[match.end() :])
                prefix.clear()
                transfer_started = loop.time()
                last_progress = transfer_started
                host_print(
                    f"receiving TWLG: {expected_bytes} bytes, {record_count} records"
                )
            else:
                payload.extend(chunk)

            now = loop.time()
            if (
                expected_bytes is not None
                and transfer_started is not None
                and len(payload) < expected_bytes
                and now - last_progress >= 5.0
            ):
                host_print(
                    _progress_line(
                        len(payload),
                        expected_bytes,
                        max(1.0e-6, now - transfer_started),
                    )
                )
                last_progress = now

            if expected_bytes is not None and len(payload) >= expected_bytes:
                payload = payload[:expected_bytes]
                transfer_finished = loop.time()
                break

        await client.stop_notify(TX_UUID)

    if expected_bytes is None:
        raise RuntimeError("timed out waiting for logdump marker")
    if len(payload) != expected_bytes:
        raise RuntimeError(
            f"short BLE dump: received {len(payload)} of {expected_bytes} bytes"
        )

    meta, _ = twlog.decode_bytes(bytes(payload), source="BLE download")
    if meta.record_count != record_count:
        raise RuntimeError(
            f"record count mismatch: marker={record_count}, header={meta.record_count}"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(payload)

    if transfer_started is not None:
        if transfer_finished is None:
            transfer_finished = time.monotonic()
        elapsed_s = max(1.0e-6, transfer_finished - transfer_started)
        host_print(
            f"download complete  elapsed={elapsed_s:.3f}s  "
            f"avg={len(payload) / elapsed_s / 1024.0:.1f} KiB/s"
        )
    host_print(
        f"saved TWLG v{meta.version}: {meta.record_count} records, "
        f"nominal={meta.duration_s:.3f} s, CRC=0x{meta.payload_crc32:08x}"
    )
    host_print(args.output)
    return 0


def download_main(argv: Sequence[str], *, session_header: bool = True) -> int:
    args = _parser().parse_args(list(argv))
    if session_header:
        print_session_header()
    try:
        return asyncio.run(_download_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        host_print(f"error: {exc}")
        return 1
