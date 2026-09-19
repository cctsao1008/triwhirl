from __future__ import annotations

import argparse
import asyncio
import json
import math
import re
from pathlib import Path
from typing import Sequence

from .. import twlog
from ..ble import DEVICE_NAME, TX_UUID, drain_queue, discover_target, send_command

MARKER = re.compile(
    rb"logdump,format=TWLG1,bytes=(\d+),record_size=(\d+),records=(\d+)\r?\n"
)


def _download_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Download a completed TWLG log over BLE")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser


def _decode_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate/decode TWLG v1 to CSV")
    parser.add_argument("input", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser


def _inspect_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a TWLG v1 runtime log")
    parser.add_argument("input", type=Path)
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    return parser


async def _download_run(args: argparse.Namespace) -> int:
    from bleak import BleakClient

    target = await discover_target(
        name=args.name,
        address=args.address,
        scan_timeout=args.scan_timeout,
    )
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
        drain_queue(queue)

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
                if record_size != twlog.RECORD_BYTES:
                    raise RuntimeError(f"unexpected TWLG record size {record_size}")
                payload.extend(prefix[match.end() :])
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

    # Validate the complete payload before committing it to disk.  This catches
    # a truncated/corrupted transfer immediately instead of deferring failure to
    # a later decode step.
    meta, _ = twlog.decode_bytes(bytes(payload), source="BLE download")
    if meta.record_count != record_count:
        raise RuntimeError(
            f"record count mismatch: marker={record_count}, header={meta.record_count}"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(payload)
    print(
        f"saved TWLG v{meta.version}: {meta.record_count} records, "
        f"{meta.duration_s:.3f} s, CRC=0x{meta.payload_crc32:08x}"
    )
    print(args.output)
    return 0


def download_main(argv: Sequence[str]) -> int:
    args = _download_parser().parse_args(list(argv))
    try:
        return asyncio.run(_download_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


def decode_main(argv: Sequence[str]) -> int:
    args = _decode_parser().parse_args(list(argv))
    try:
        meta, payload = twlog.read(args.input)
        twlog.write_csv(args.output, payload)
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}")
        return 1

    print(
        f"TWLG v{meta.version}: {meta.record_count} records, "
        f"Ts={meta.sample_period_us} us, dropped={meta.dropped_records}, "
        f"CRC=0x{meta.payload_crc32:08x}"
    )
    print(args.output)
    return 0


def inspect_main(argv: Sequence[str]) -> int:
    args = _inspect_parser().parse_args(list(argv))
    try:
        meta, payload = twlog.read(args.input)
        stats = twlog.summarize(payload)
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}")
        return 1

    result = {
        "format": f"TWLG{meta.version}",
        "records": meta.record_count,
        "record_bytes": meta.record_size,
        "sample_period_us": meta.sample_period_us,
        "sample_rate_hz": meta.sample_rate_hz,
        "duration_s": meta.duration_s,
        "dropped_records": meta.dropped_records,
        "payload_crc32": f"0x{meta.payload_crc32:08x}",
        "header_flags": f"0x{meta.flags:08x}",
        "theta_min_deg": math.degrees(stats.theta_min_rad),
        "theta_max_deg": math.degrees(stats.theta_max_rad),
        "max_abs_theta_rate_rad_s": stats.max_abs_theta_rate_rad_s,
        "max_abs_wheel_rate_rad_s": stats.max_abs_wheel_rate_rad_s,
        "vq_min_v": stats.vq_min_v,
        "vq_max_v": stats.vq_max_v,
        "accel_weight_min": stats.accel_weight_min,
        "accel_weight_max": stats.accel_weight_max,
        "fault_or": f"0x{stats.fault_or:08x}",
        "faulted_records": stats.faulted_records,
        "record_flags_or": f"0x{stats.flags_or:04x}",
    }

    if args.json:
        print(json.dumps(result, indent=2))
        return 0

    print(f"{args.input}")
    print(
        f"TWLG v{meta.version}  records={meta.record_count}  "
        f"Ts={meta.sample_period_us} us ({meta.sample_rate_hz:.1f} Hz)  "
        f"duration={meta.duration_s:.3f} s"
    )
    print(
        f"dropped={meta.dropped_records}  CRC=0x{meta.payload_crc32:08x}  "
        f"header_flags=0x{meta.flags:08x}"
    )
    print(
        f"theta=[{result['theta_min_deg']:+.2f}, {result['theta_max_deg']:+.2f}] deg  "
        f"max|theta_rate|={stats.max_abs_theta_rate_rad_s:.3f} rad/s"
    )
    print(
        f"max|wheel_rate|={stats.max_abs_wheel_rate_rad_s:.3f} rad/s  "
        f"Vq=[{stats.vq_min_v:+.3f}, {stats.vq_max_v:+.3f}] V"
    )
    print(
        f"fault_or=0x{stats.fault_or:08x}  faulted_records={stats.faulted_records}  "
        f"record_flags_or=0x{stats.flags_or:04x}"
    )
    return 0
