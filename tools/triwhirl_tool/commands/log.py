from __future__ import annotations

import argparse
import asyncio
import json
import math
import re
import time
from pathlib import Path
from typing import Sequence

from .. import twlog
from ..ble import (
    DEVICE_NAME,
    TX_UUID,
    BleLineTransport,
    drain_queue,
    discover_target,
    send_command,
)

MARKER = re.compile(
    rb"logdump,format=TWLG1,bytes=(\d+),record_size=(\d+),records=(\d+)\r?\n"
)


def _add_ble_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)


def _download_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Download a completed TWLG log over BLE")
    parser.add_argument("-o", "--output", type=Path, required=True)
    _add_ble_args(parser)
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


def _status_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read firmware TWLG logger status over BLE")
    _add_ble_args(parser)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser


def _prepare_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pre-erase/prepare the firmware TWLG region before an experiment"
    )
    parser.add_argument("seconds", nargs="?", type=float, default=45.0)
    _add_ble_args(parser)
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=90.0,
        help="maximum time to wait for background sector erase to reach state=ready",
    )
    parser.add_argument("--no-wait", action="store_true")
    return parser


def _simple_control_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    _add_ble_args(parser)
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser


def _critical_parser() -> argparse.ArgumentParser:
    parser = _simple_control_parser(
        "Pause/resume flash programming while retaining 1 kHz records in SRAM"
    )
    parser.add_argument("mode", choices=("on", "off"))
    return parser


def _normalize_console_line(line: str) -> str:
    text = line.strip()
    while text.startswith(">"):
        text = text[1:].lstrip()
    return text


def _parse_key_values(line: str, prefix: str) -> dict[str, str]:
    normalized = _normalize_console_line(line)
    marker = prefix + ","
    index = normalized.find(marker)
    if index < 0:
        raise RuntimeError(f"unexpected firmware response: {line}")
    result: dict[str, str] = {}
    for item in normalized[index + len(marker) :].split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value
    return result


async def _open_line_transport(args: argparse.Namespace):
    from bleak import BleakClient

    target = await discover_target(
        name=args.name,
        address=args.address,
        scan_timeout=args.scan_timeout,
    )
    client = BleakClient(target)
    await client.connect()
    if not client.is_connected:
        raise RuntimeError("BLE connection failed")
    transport = BleLineTransport(client)
    await client.start_notify(TX_UUID, transport.on_notify)
    await asyncio.sleep(0.2)
    await transport.send("telemetry off")
    await asyncio.sleep(0.05)
    transport.drain()
    return client, transport


async def _close_line_transport(client, transport: BleLineTransport) -> None:
    try:
        if client.is_connected:
            await client.stop_notify(TX_UUID)
    finally:
        if client.is_connected:
            await client.disconnect()


async def _wait_console(
    transport: BleLineTransport,
    *,
    prefixes: tuple[str, ...],
    timeout_s: float,
) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = max(0.001, deadline - time.monotonic())
        line = await transport.read_line(min(0.5, remaining))
        if not line:
            continue
        normalized = _normalize_console_line(line)
        err_index = normalized.find("ERR ")
        if err_index >= 0:
            raise RuntimeError(normalized[err_index:])
        for prefix in prefixes:
            index = normalized.find(prefix)
            if index >= 0:
                return normalized[index:]
    raise RuntimeError(f"timed out waiting for firmware response {prefixes}")


async def _request_log_status(
    transport: BleLineTransport,
    timeout_s: float = 3.0,
) -> tuple[str, dict[str, str]]:
    await transport.send("log status")
    line = await _wait_console(transport, prefixes=("log,",), timeout_s=timeout_s)
    return line, _parse_key_values(line, "log")


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


async def _status_run(args: argparse.Namespace) -> int:
    client, transport = await _open_line_transport(args)
    try:
        line, values = await _request_log_status(transport, args.timeout)
        if args.json:
            print(json.dumps(values, indent=2))
        else:
            print(line)
        return 0
    finally:
        await _close_line_transport(client, transport)


def status_main(argv: Sequence[str]) -> int:
    args = _status_parser().parse_args(list(argv))
    try:
        return asyncio.run(_status_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


async def _prepare_run(args: argparse.Namespace) -> int:
    if not math.isfinite(args.seconds) or args.seconds <= 0.0:
        raise RuntimeError("seconds must be finite and > 0")
    client, transport = await _open_line_transport(args)
    try:
        await transport.send(f"log prepare {args.seconds:.9g}")
        line = await _wait_console(
            transport,
            prefixes=("OK log prepare",),
            timeout_s=5.0,
        )
        print(line)
        if args.no_wait:
            return 0

        deadline = time.monotonic() + args.wait_timeout
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
                return 0
            if state in {"error", "unavailable"}:
                raise RuntimeError(f"logger entered state={state}")
            await asyncio.sleep(0.25)
        raise RuntimeError(
            f"timed out after {args.wait_timeout:.1f}s waiting for logger state=ready"
        )
    finally:
        await _close_line_transport(client, transport)


def prepare_main(argv: Sequence[str]) -> int:
    args = _prepare_parser().parse_args(list(argv))
    try:
        return asyncio.run(_prepare_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


async def _simple_log_control_run(
    args: argparse.Namespace,
    command: str,
    ok_prefix: str,
    *,
    wait_complete: bool = False,
) -> int:
    client, transport = await _open_line_transport(args)
    try:
        await transport.send(command)
        line = await _wait_console(
            transport,
            prefixes=(ok_prefix,),
            timeout_s=args.timeout,
        )
        print(line)
        if not wait_complete:
            return 0

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            _line, status = await _request_log_status(transport, 3.0)
            state = status.get("state", "unknown")
            buffered = status.get("buffered_bytes", "?")
            if state == "complete":
                print(
                    f"log state=complete records={status.get('records_written', '?')} "
                    f"logical_bytes={status.get('logical_bytes', '?')} dropped={status.get('dropped_records', '?')}"
                )
                return 0
            if state == "error":
                raise RuntimeError("logger entered state=error while stopping")
            print(f"log state={state} buffered_bytes={buffered}")
            await asyncio.sleep(0.1)
        raise RuntimeError("timed out waiting for logger state=complete")
    finally:
        await _close_line_transport(client, transport)


def start_main(argv: Sequence[str]) -> int:
    args = _simple_control_parser("Start the prepared 1 kHz firmware TWLG logger").parse_args(
        list(argv)
    )
    try:
        return asyncio.run(
            _simple_log_control_run(args, "log start", "OK log start")
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


def stop_main(argv: Sequence[str]) -> int:
    parser = _simple_control_parser(
        "Stop the firmware TWLG logger and wait for SRAM/Flash finalization"
    )
    parser.set_defaults(timeout=15.0)
    args = parser.parse_args(list(argv))
    try:
        return asyncio.run(
            _simple_log_control_run(
                args,
                "log stop",
                "OK log stopping",
                wait_complete=True,
            )
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


def critical_main(argv: Sequence[str]) -> int:
    args = _critical_parser().parse_args(list(argv))
    command = f"log critical {args.mode}"
    ok_prefix = f"OK log critical {args.mode}"
    try:
        return asyncio.run(_simple_log_control_run(args, command, ok_prefix))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
