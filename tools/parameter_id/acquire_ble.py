#!/usr/bin/env python3
"""Acquire TriWhirl identification telemetry over native BLE GATT.

This is the untethered companion to acquire.py. It uses the same line-oriented
firmware command/telemetry protocol over the existing NimBLE service, so the
unit can run from its battery without a USB cable disturbing body motion.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

from bleak import BleakClient, BleakScanner

SERVICE_UUID = "54f10000-8f4d-4f3a-b691-54524957484c"
RX_UUID = "54f10001-8f4d-4f3a-b691-54524957484c"
TX_UUID = "54f10002-8f4d-4f3a-b691-54524957484c"
DEVICE_NAME = "TriWhirl"
SCHEMA_VERSION = 2
DEFAULT_MOTOR_CONFIG = Path("artifacts/motor-config.json")

TELEMETRY_FIELDS = (
    "t_us", "mode", "vq_v", "e_angle_rad", "e_hz", "status_ok",
    "sample_ok", "mag", "raw", "unwrapped_count", "angle_rad",
    "unwrapped_rad", "vel_rad_s", "vel_inst_rad_s", "vel_valid",
    "read_errors", "imu_ok", "ax", "ay", "az", "gx", "gy", "gz",
    "imu_read_errors", "attitude_ok", "theta_rad", "theta_rate_rad_s",
    "accel_weight", "loop_exec_us", "loop_max_exec_us", "loop_overruns",
    "fault_mask",
)


@dataclass(frozen=True)
class Segment:
    vq_v: float
    duration_s: float


@dataclass(frozen=True)
class MotorConfig:
    pole_pairs: int
    sensor_dir: int
    offset_rad: float


def parse_segment(text: str) -> Segment:
    try:
        vq_text, duration_text = text.split(":", 1)
        vq = float(vq_text)
        duration = float(duration_text)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError(
            "segment must be VQ:DURATION, for example 0.25:0.8"
        ) from exc
    if not math.isfinite(vq) or not math.isfinite(duration) or duration <= 0.0:
        raise argparse.ArgumentTypeError("segment values must be finite and duration > 0")
    return Segment(vq, duration)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a TriWhirl identification profile over BLE without a USB tether."
    )
    parser.add_argument(
        "--segment", action="append", type=parse_segment, required=True,
        help="Vq volts and duration seconds as VQ:DURATION; repeat for a profile",
    )
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--pre-roll", type=float, default=1.0)
    parser.add_argument("--post-roll", type=float, default=1.0)
    parser.add_argument("--ready-timeout", type=float, default=10.0)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--name", default=DEVICE_NAME, help="BLE device name")
    parser.add_argument("--address", default=None, help="optional BLE address/device identifier")
    parser.add_argument("--auto-calibrate", action="store_true")
    parser.add_argument("--calibration-timeout", type=float, default=30.0)
    parser.add_argument(
        "--motor-config", type=Path, default=DEFAULT_MOTOR_CONFIG,
        help=f"reusable motor electrical config JSON (default: {DEFAULT_MOTOR_CONFIG})",
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def parse_key_values(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in line.split(",")[1:]:
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value
    return result


def motor_config_from_status(status: dict[str, str]) -> MotorConfig | None:
    if status.get("config") != "1":
        return None
    try:
        config = MotorConfig(
            pole_pairs=int(status["pole_pairs"]),
            sensor_dir=int(status["sensor_dir"]),
            offset_rad=float(status["offset_rad"]),
        )
    except (KeyError, TypeError, ValueError):
        return None
    if (
        not 1 <= config.pole_pairs <= 64
        or config.sensor_dir not in (-1, 1)
        or not math.isfinite(config.offset_rad)
    ):
        return None
    return config


def load_motor_config(path: Path) -> MotorConfig | None:
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    config = MotorConfig(
        pole_pairs=int(data["pole_pairs"]),
        sensor_dir=int(data["sensor_dir"]),
        offset_rad=float(data["offset_rad"]),
    )
    if (
        not 1 <= config.pole_pairs <= 64
        or config.sensor_dir not in (-1, 1)
        or not math.isfinite(config.offset_rad)
    ):
        raise RuntimeError(f"invalid motor config values in {path}")
    return config


def save_motor_config(path: Path, config: MotorConfig, source: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "format": "triwhirl-motor-config-v1",
        "pole_pairs": config.pole_pairs,
        "sensor_dir": config.sensor_dir,
        "offset_rad": config.offset_rad,
        "source": source,
        "saved": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def parse_telemetry(line: str) -> list[str] | None:
    if not line.startswith("telemetry,"):
        return None
    values = line.split(",")[1:]
    if len(values) != len(TELEMETRY_FIELDS):
        raise RuntimeError(
            f"telemetry schema mismatch: got {len(values)} fields, expected {len(TELEMETRY_FIELDS)}"
        )
    return values


def telemetry_fault_mask(values: list[str]) -> int:
    return int(values[TELEMETRY_FIELDS.index("fault_mask")], 0)


def telemetry_ready(values: list[str]) -> bool:
    return (
        values[TELEMETRY_FIELDS.index("attitude_ok")] == "1"
        and values[TELEMETRY_FIELDS.index("vel_valid")] == "1"
        and telemetry_fault_mask(values) == 0
    )


class BleLineTransport:
    def __init__(self, client: BleakClient):
        self.client = client
        self.lines: asyncio.Queue[str] = asyncio.Queue()
        self.buffer = ""

    def on_notify(self, _sender, data: bytearray) -> None:
        self.buffer += bytes(data).decode("utf-8", errors="replace")
        while True:
            newline = self.buffer.find("\n")
            if newline < 0:
                return
            line = self.buffer[:newline].rstrip("\r")
            self.buffer = self.buffer[newline + 1:]
            if line:
                self.lines.put_nowait(line)

    async def send(self, command: str) -> None:
        payload = (command.rstrip("\r\n") + "\n").encode("ascii")
        # 20-byte chunks work even before a larger ATT MTU has been negotiated.
        for offset in range(0, len(payload), 20):
            await self.client.write_gatt_char(
                RX_UUID, payload[offset:offset + 20], response=False
            )

    async def read_line(self, timeout_s: float) -> str | None:
        try:
            return await asyncio.wait_for(self.lines.get(), timeout_s)
        except asyncio.TimeoutError:
            return None


async def wait_for_status(transport: BleLineTransport, timeout_s: float = 3.0) -> dict[str, str]:
    await transport.send("status")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = await transport.read_line(min(0.5, deadline - time.monotonic()))
        if not line:
            continue
        if line.startswith("FAULT,"):
            raise RuntimeError(line)
        if line.startswith("status,"):
            return parse_key_values(line)
    raise RuntimeError("timed out waiting for firmware status over BLE")


async def apply_motor_config(
    transport: BleLineTransport, config: MotorConfig
) -> dict[str, str]:
    await transport.send(
        f"motor config {config.pole_pairs} {config.sensor_dir} {config.offset_rad:.9g}"
    )
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        line = await transport.read_line(min(0.5, deadline - time.monotonic()))
        if not line:
            continue
        if line.startswith("FAULT,") or line.startswith("ERR"):
            raise RuntimeError(f"firmware rejected motor config: {line}")
        if line.startswith("OK motor config"):
            status = await wait_for_status(transport)
            if motor_config_from_status(status) is None:
                raise RuntimeError("firmware did not retain the supplied motor config")
            return status
    raise RuntimeError("timed out applying motor config over BLE")


async def ensure_motor_config(
    transport: BleLineTransport,
    config_path: Path,
    auto_calibrate: bool,
    timeout_s: float,
) -> tuple[dict[str, str], MotorConfig | None]:
    status = await wait_for_status(transport)
    config = motor_config_from_status(status)
    if config is not None:
        save_motor_config(config_path, config, "firmware-status-ble")
        return status, config

    saved = load_motor_config(config_path)
    if saved is not None:
        print(f"loading motor config from {config_path}")
        return await apply_motor_config(transport, saved), saved

    if not auto_calibrate:
        # A zero-Vq experiment does not need an electrical motor configuration.
        return status, None

    print("motor config absent; starting firmware calibration over BLE")
    await transport.send("fault clear")
    await transport.send("motor calibrate")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = await transport.read_line(min(0.5, deadline - time.monotonic()))
        if not line:
            continue
        print(line)
        if line.startswith("FAULT,") or line.startswith("ERR motor calibration"):
            raise RuntimeError(f"motor calibration failed: {line}")
        if line.startswith("OK motor calibrated"):
            status = await wait_for_status(transport)
            config = motor_config_from_status(status)
            if config is None:
                raise RuntimeError("calibration completed but firmware config is invalid")
            save_motor_config(config_path, config, "firmware-calibration-ble")
            return status, config
    raise RuntimeError("motor calibration timed out")


async def capture_until(
    transport: BleLineTransport,
    writer: csv.writer,
    deadline: float,
    phase: str,
    rows: list[int],
    require_ready: bool = False,
) -> bool:
    ready = False
    while time.monotonic() < deadline:
        line = await transport.read_line(min(0.25, max(0.001, deadline - time.monotonic())))
        if not line:
            continue
        if line.startswith("FAULT,"):
            raise RuntimeError(line)
        values = parse_telemetry(line)
        if values is None:
            continue
        mask = telemetry_fault_mask(values)
        if mask != 0:
            raise RuntimeError(f"firmware fault_mask became nonzero: 0x{mask:08x}")
        writer.writerow((SCHEMA_VERSION, phase, *values))
        rows[0] += 1
        ready = ready or telemetry_ready(values)
        if require_ready and ready:
            return True
    return ready


async def discover_target(args: argparse.Namespace):
    if args.address:
        return args.address
    print(f"scanning for BLE device {args.name!r} ...")
    device = await BleakScanner.find_device_by_name(args.name, timeout=args.scan_timeout)
    if device is None:
        raise RuntimeError(f"BLE device {args.name!r} not found")
    print(f"found {device.name or args.name}: {device.address}")
    return device


async def run(args: argparse.Namespace) -> int:
    if args.repeat < 1:
        raise RuntimeError("--repeat must be >= 1")
    if args.pre_roll < 0.0 or args.post_roll < 0.0 or args.ready_timeout <= 0.0:
        raise RuntimeError("capture durations must be non-negative and ready timeout > 0")

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = output.with_suffix(output.suffix + ".json")
    rows = [0]
    started_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    completed = False
    motor_config: MotorConfig | None = None
    target = await discover_target(args)

    try:
        async with BleakClient(target) as client:
            if not client.is_connected:
                raise RuntimeError("BLE connection failed")
            transport = BleLineTransport(client)
            await client.start_notify(TX_UUID, transport.on_notify)
            await asyncio.sleep(0.2)

            await transport.send("motor stop")
            await transport.send("telemetry off")

            status, motor_config = await ensure_motor_config(
                transport, args.motor_config, args.auto_calibrate,
                args.calibration_timeout,
            )
            if int(status.get("fault_mask", "0"), 0) != 0:
                await transport.send("fault clear")
                status = await wait_for_status(transport)
                if int(status.get("fault_mask", "0"), 0) != 0:
                    raise RuntimeError(
                        f"firmware fault remains latched: {status.get('fault_mask')}"
                    )

            needs_motor_config = any(abs(segment.vq_v) >= 1.0e-9 for segment in args.segment)
            if needs_motor_config and motor_config is None:
                raise RuntimeError(
                    "nonzero Vq profile requires motor config; restore artifacts/motor-config.json "
                    "or use --auto-calibrate"
                )

            with output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("schema_version", "phase", *TELEMETRY_FIELDS))
                await transport.send("telemetry on")

                ready_deadline = time.monotonic() + args.ready_timeout
                if not await capture_until(
                    transport, writer, ready_deadline, "ready", rows, require_ready=True
                ):
                    raise RuntimeError("attitude/wheel state did not become ready before timeout")

                if args.pre_roll > 0.0:
                    await capture_until(
                        transport, writer, time.monotonic() + args.pre_roll, "pre", rows
                    )

                for repetition in range(args.repeat):
                    for index, segment in enumerate(args.segment, start=1):
                        phase = f"r{repetition + 1}_s{index}"
                        if abs(segment.vq_v) < 1.0e-9:
                            await transport.send("motor stop")
                        else:
                            await transport.send(f"motor vq {segment.vq_v:.9g}")
                        await capture_until(
                            transport, writer,
                            time.monotonic() + segment.duration_s,
                            phase, rows,
                        )

                await transport.send("motor stop")
                if args.post_roll > 0.0:
                    await capture_until(
                        transport, writer, time.monotonic() + args.post_roll,
                        "post", rows,
                    )
                completed = True

            try:
                await transport.send("motor stop")
                await transport.send("telemetry off")
            finally:
                await client.stop_notify(TX_UUID)

    finally:
        metadata = {
            "format": "triwhirl-identification-run-v1",
            "telemetry_schema_version": SCHEMA_VERSION,
            "transport": "ble",
            "device_name": args.name,
            "device_address": args.address,
            "started": started_wall,
            "completed": completed,
            "repeat": args.repeat,
            "pre_roll_s": args.pre_roll,
            "post_roll_s": args.post_roll,
            "motor_config_file": str(args.motor_config),
            "motor_config": None if motor_config is None else {
                "pole_pairs": motor_config.pole_pairs,
                "sensor_dir": motor_config.sensor_dir,
                "offset_rad": motor_config.offset_rad,
            },
            "segments": [
                {"vq_v": segment.vq_v, "duration_s": segment.duration_s}
                for segment in args.segment
            ],
            "telemetry_rows": rows[0],
            "csv": str(output),
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

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
