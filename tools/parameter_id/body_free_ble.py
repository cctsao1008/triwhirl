#!/usr/bin/env python3
"""Acquire an untethered TriWhirl body free-response over BLE.

The unit is battery powered and physically untethered. The script calibrates the
MPU6050 gyro only after the BLE link is established, resets the attitude estimate
from gravity, then records a zero-Vq body-motion window.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

from acquire_ble import (
    DEVICE_NAME,
    SCHEMA_VERSION,
    TELEMETRY_FIELDS,
    TX_UUID,
    BleLineTransport,
    capture_until,
    parse_key_values,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record a battery-powered free body response over BLE."
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument("--start-delay", type=float, default=3.0)
    parser.add_argument("--ready-timeout", type=float, default=10.0)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


async def read_until_prefix(
    transport: BleLineTransport,
    prefix: str,
    timeout_s: float,
) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = await transport.read_line(min(0.5, deadline - time.monotonic()))
        if not line:
            continue
        if line.startswith("FAULT,") or line.startswith("ERR"):
            raise RuntimeError(line)
        if line.startswith(prefix):
            return line
    raise RuntimeError(f"timed out waiting for {prefix!r}")


async def wait_for_gyro_calibration(
    transport: BleLineTransport,
    samples: int,
) -> str:
    # The firmware samples the IMU from the real-time loop, but the effective
    # sample cadence can be slower than the nominal 1 kHz while BLE/console work
    # is active. Also, a single completion notification can be lost without the
    # calibration itself failing. Poll `imu status` as an authoritative fallback
    # instead of treating one missing console line as a failed calibration.
    timeout_s = max(15.0, samples * 0.03)
    deadline = time.monotonic() + timeout_s
    next_status_poll = time.monotonic() + 0.5
    last_status: dict[str, str] | None = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        wait_s = min(0.25, max(0.001, deadline - now))
        line = await transport.read_line(wait_s)
        if line:
            if line.startswith("FAULT,") or line.startswith("ERR"):
                raise RuntimeError(line)
            if line.startswith("OK imu gyro calibration bx="):
                return line
            if line.startswith("imu,"):
                status = parse_key_values(line)
                last_status = status
                if (
                    status.get("ready") == "1"
                    and status.get("sample_ok") == "1"
                    and status.get("bias_valid") == "1"
                    and status.get("calibrating") == "0"
                ):
                    return (
                        "OK imu gyro calibration status "
                        f"bx={status.get('bx', '?')} "
                        f"by={status.get('by', '?')} "
                        f"bz={status.get('bz', '?')} rad_s"
                    )

        now = time.monotonic()
        if now >= next_status_poll:
            await transport.send("imu status")
            next_status_poll = now + 0.5

    if last_status is None:
        detail = "no imu status received"
    else:
        detail = (
            f"ready={last_status.get('ready')} "
            f"sample_ok={last_status.get('sample_ok')} "
            f"bias_valid={last_status.get('bias_valid')} "
            f"calibrating={last_status.get('calibrating')} "
            f"read_errors={last_status.get('read_errors')}"
        )
    raise RuntimeError(
        f"gyro calibration did not complete within {timeout_s:.1f}s ({detail})"
    )


async def prepare_imu(
    transport: BleLineTransport,
    samples: int,
) -> str:
    samples = max(50, min(5000, samples))
    print(f"keep the unit still: calibrating gyro with {samples} samples ...")
    await transport.send(f"imu calibrate {samples}")
    await read_until_prefix(transport, "OK imu gyro calibration started", 3.0)
    result = await wait_for_gyro_calibration(transport, samples)
    await transport.send("attitude reset")
    await read_until_prefix(
        transport, "OK attitude reset from accelerometer", 3.0
    )
    print(result)
    return result


async def discover(args: argparse.Namespace):
    if args.address:
        return args.address
    print(f"scanning for BLE device {args.name!r} ...")
    device = await BleakScanner.find_device_by_name(
        args.name, timeout=args.scan_timeout
    )
    if device is None:
        raise RuntimeError(f"BLE device {args.name!r} not found")
    print(f"found {device.name or args.name}: {device.address}")
    return device


async def run(args: argparse.Namespace) -> int:
    if args.duration <= 0.0:
        raise RuntimeError("--duration must be > 0")
    if args.start_delay < 0.0:
        raise RuntimeError("--start-delay must be >= 0")

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = output.with_suffix(output.suffix + ".json")
    target = await discover(args)
    resolved_address = getattr(target, "address", None) or args.address
    rows = [0]
    started_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    completed = False
    calibration_line = None

    try:
        async with BleakClient(target) as client:
            if not client.is_connected:
                raise RuntimeError("BLE connection failed")
            transport = BleLineTransport(client)
            await client.start_notify(TX_UUID, transport.on_notify)
            await asyncio.sleep(0.2)

            try:
                await transport.send("motor stop")
                await transport.send("telemetry off")
                calibration_line = await prepare_imu(
                    transport, args.imu_samples
                )

                if args.start_delay > 0.0:
                    print(
                        f"IMU ready. Free the body and prepare a small displacement; "
                        f"capture starts in {args.start_delay:.1f} s."
                    )
                    await asyncio.sleep(args.start_delay)

                with output.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(
                        ("schema_version", "phase", *TELEMETRY_FIELDS)
                    )
                    await transport.send("telemetry on")

                    ready_deadline = time.monotonic() + args.ready_timeout
                    if not await capture_until(
                        transport,
                        writer,
                        ready_deadline,
                        "ready",
                        rows,
                        require_ready=True,
                    ):
                        raise RuntimeError(
                            "attitude/wheel state did not become ready before timeout"
                        )

                    print(
                        f"START: gently displace/release the body now; "
                        f"recording {args.duration:.1f} s"
                    )
                    await capture_until(
                        transport,
                        writer,
                        time.monotonic() + args.duration,
                        "body_free",
                        rows,
                    )
                    completed = True
                    print("capture complete")
            finally:
                if client.is_connected:
                    for command in ("motor stop", "telemetry off"):
                        try:
                            await transport.send(command)
                        except Exception:
                            pass
                    try:
                        await client.stop_notify(TX_UUID)
                    except Exception:
                        pass
    finally:
        metadata = {
            "format": "triwhirl-body-free-run-v1",
            "telemetry_schema_version": SCHEMA_VERSION,
            "transport": "ble",
            "device_name": args.name,
            "device_address": resolved_address,
            "started": started_wall,
            "completed": completed,
            "duration_s": args.duration,
            "imu_calibration_samples": max(50, min(5000, args.imu_samples)),
            "imu_calibration_result": calibration_line,
            "start_delay_s": args.start_delay,
            "telemetry_rows": rows[0],
            "csv": str(output),
        }
        metadata_path.write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

    print(f"saved {rows[0]} telemetry rows -> {output}")
    print(f"saved run metadata -> {metadata_path}")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(run(args))
    except (RuntimeError, OSError, json.JSONDecodeError) as exc:
        print(f"error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
