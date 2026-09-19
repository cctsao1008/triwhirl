#!/usr/bin/env python3
"""Download a completed TWLG binary log over native BLE GATT.

BLE is deliberately used only after the realtime experiment has finished.  The
firmware emits an ASCII marker with the exact binary length, followed by that
many TWLG bytes.  This tool ignores command echo/text before the marker and
writes the binary stream verbatim.
"""

from __future__ import annotations

import argparse
import asyncio
import re
from pathlib import Path

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "TriWhirl"
RX_UUID = "54f10001-8f4d-4f3a-b691-54524957484c"
TX_UUID = "54f10002-8f4d-4f3a-b691-54524957484c"
MARKER = re.compile(
    rb"logdump,format=TWLG1,bytes=(\d+),record_size=(\d+),records=(\d+)\r?\n"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Download completed TWLG log over BLE")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args()


async def discover(args: argparse.Namespace):
    if args.address:
        return args.address
    print(f"scanning for BLE device {args.name!r} ...")
    device = await BleakScanner.find_device_by_name(args.name, timeout=args.scan_timeout)
    if device is None:
        raise RuntimeError(f"BLE device {args.name!r} not found")
    print(f"found {device.name or args.name}: {device.address}")
    return device


async def send_command(client: BleakClient, command: str) -> None:
    payload = (command.rstrip("\r\n") + "\n").encode("ascii")
    for offset in range(0, len(payload), 20):
        await client.write_gatt_char(RX_UUID, payload[offset:offset + 20], response=False)


async def run(args: argparse.Namespace) -> int:
    target = await discover(args)
    queue: asyncio.Queue[bytes] = asyncio.Queue()

    def on_notify(_sender, data: bytearray) -> None:
        queue.put_nowait(bytes(data))

    async with BleakClient(target) as client:
        if not client.is_connected:
            raise RuntimeError("BLE connection failed")
        await client.start_notify(TX_UUID, on_notify)
        await asyncio.sleep(0.2)
        await send_command(client, "telemetry off")
        await asyncio.sleep(0.05)
        while not queue.empty():
            queue.get_nowait()

        await send_command(client, "log dump")
        loop = asyncio.get_running_loop()
        deadline = loop.time() + args.timeout
        prefix = bytearray()
        payload = bytearray()
        expected_bytes: int | None = None
        record_size = 0
        record_count = 0

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
                if record_size != 32:
                    raise RuntimeError(f"unexpected TWLG record size {record_size}")
                payload.extend(prefix[match.end():])
                prefix.clear()
                print(
                    f"receiving TWLG: {expected_bytes} bytes, "
                    f"{record_count} records"
                )
            else:
                payload.extend(chunk)

            if expected_bytes is not None and len(payload) >= expected_bytes:
                payload = payload[:expected_bytes]
                break

        await client.stop_notify(TX_UUID)

    if expected_bytes is None:
        raise RuntimeError("timed out waiting for logdump marker")
    if len(payload) != expected_bytes:
        raise RuntimeError(
            f"short BLE dump: received {len(payload)} of {expected_bytes} bytes"
        )
    if payload[:4] != b"TWLG":
        raise RuntimeError(f"download does not begin with TWLG magic: {payload[:4]!r}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(payload)
    print(args.output)
    return 0


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(run(args))
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
