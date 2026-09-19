#!/usr/bin/env python3
"""Acquire a controlled TriWhirl identification run over the CH340 UART.

The tool executes an explicit Vq profile, records telemetry schema v2, aborts on
any latched firmware fault, and always commands the motor to stop on exit.
Hardware limits and fault authority remain in the firmware; this script does
not invent safety thresholds.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial

SCHEMA_VERSION = 2
DEFAULT_MOTOR_CONFIG = Path("artifacts/motor-config.json")
TELEMETRY_FIELDS = (
    "t_us",
    "mode",
    "vq_v",
    "e_angle_rad",
    "e_hz",
    "status_ok",
    "sample_ok",
    "mag",
    "raw",
    "unwrapped_count",
    "angle_rad",
    "unwrapped_rad",
    "vel_rad_s",
    "vel_inst_rad_s",
    "vel_valid",
    "read_errors",
    "imu_ok",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "imu_read_errors",
    "attitude_ok",
    "theta_rad",
    "theta_rate_rad_s",
    "accel_weight",
    "loop_exec_us",
    "loop_max_exec_us",
    "loop_overruns",
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


def default_output_path() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path(f"triwhirl-id-{stamp}.csv")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an explicit TriWhirl Vq excitation profile and record telemetry."
    )
    parser.add_argument("port", help="serial port, for example COM28")
    parser.add_argument(
        "--segment",
        action="append",
        type=parse_segment,
        required=True,
        help="Vq volts and duration seconds as VQ:DURATION; repeat for a profile",
    )
    parser.add_argument("--repeat", type=int, default=1, help="profile repetitions")
    parser.add_argument("--pre-roll", type=float, default=1.0, help="stopped capture before profile [s]")
    parser.add_argument("--post-roll", type=float, default=1.0, help="stopped capture after profile [s]")
    parser.add_argument("--ready-timeout", type=float, default=10.0, help="wait for valid attitude/wheel telemetry [s]")
    parser.add_argument("--auto-calibrate", action="store_true", help="run the existing firmware motor calibration if no reusable config exists")
    parser.add_argument("--calibration-timeout", type=float, default=30.0)
    parser.add_argument(
        "--motor-config",
        type=Path,
        default=DEFAULT_MOTOR_CONFIG,
        help=f"reusable motor electrical config JSON (default: {DEFAULT_MOTOR_CONFIG})",
    )
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-o", "--output", type=Path, default=None)
    return parser.parse_args()


def write_command(port: serial.Serial, command: str) -> None:
    port.write((command.rstrip("\r\n") + "\r\n").encode("ascii"))
    port.flush()


def read_line(port: serial.Serial) -> str | None:
    raw = port.readline()
    if not raw:
        return None
    return raw.decode("utf-8", errors="replace").strip()


def parse_key_values(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in line.split(",")[1:]:
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value
    return result


def wait_for_status(port: serial.Serial, timeout_s: float = 3.0) -> dict[str, str]:
    write_command(port, "status")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(port)
        if not line:
            continue
        if line.startswith("FAULT,"):
            raise RuntimeError(line)
        if line.startswith("status,"):
            return parse_key_values(line)
    raise RuntimeError("timed out waiting for firmware status")


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
        config.pole_pairs < 1
        or config.pole_pairs > 64
        or config.sensor_dir not in (-1, 1)
        or not math.isfinite(config.offset_rad)
    ):
        return None
    return config


def load_motor_config(path: Path) -> MotorConfig | None:
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        config = MotorConfig(
            pole_pairs=int(data["pole_pairs"]),
            sensor_dir=int(data["sensor_dir"]),
            offset_rad=float(data["offset_rad"]),
        )
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"invalid motor config file {path}: {exc}") from exc
    if (
        config.pole_pairs < 1
        or config.pole_pairs > 64
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


def apply_motor_config(port: serial.Serial, config: MotorConfig) -> dict[str, str]:
    write_command(
        port,
        f"motor config {config.pole_pairs} {config.sensor_dir} {config.offset_rad:.9g}",
    )
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        line = read_line(port)
        if not line:
            continue
        if line.startswith("FAULT,") or line.startswith("ERR"):
            raise RuntimeError(f"firmware rejected motor config: {line}")
        if line.startswith("OK motor config"):
            status = wait_for_status(port)
            if motor_config_from_status(status) is None:
                raise RuntimeError("firmware did not retain the supplied motor config")
            return status
    raise RuntimeError("timed out applying motor config")


def ensure_motor_config(
    port: serial.Serial,
    config_path: Path,
    auto_calibrate: bool,
    timeout_s: float,
) -> tuple[dict[str, str], MotorConfig]:
    status = wait_for_status(port)
    config = motor_config_from_status(status)
    if config is not None:
        save_motor_config(config_path, config, "firmware-status")
        return status, config

    saved = load_motor_config(config_path)
    if saved is not None:
        print(f"loading motor config from {config_path}")
        status = apply_motor_config(port, saved)
        return status, saved

    if not auto_calibrate:
        raise RuntimeError(
            f"motor electrical config is absent and {config_path} does not exist; "
            "rerun with --auto-calibrate once"
        )

    print("motor config absent; starting firmware calibration")
    write_command(port, "fault clear")
    write_command(port, "motor calibrate")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(port)
        if not line:
            continue
        print(line)
        if line.startswith("FAULT,") or line.startswith("ERR motor calibration"):
            raise RuntimeError(f"motor calibration failed: {line}")
        if line.startswith("OK motor calibrated"):
            status = wait_for_status(port)
            config = motor_config_from_status(status)
            if config is None:
                raise RuntimeError("calibration completed but firmware config is invalid")
            save_motor_config(config_path, config, "firmware-calibration")
            print(f"saved reusable motor config -> {config_path}")
            return status, config
    raise RuntimeError("motor calibration timed out")


def parse_telemetry(line: str) -> list[str] | None:
    if not line.startswith("telemetry,"):
        return None
    values = line.split(",")[1:]
    if len(values) != len(TELEMETRY_FIELDS):
        raise RuntimeError(
            f"telemetry schema mismatch: got {len(values)} fields, expected {len(TELEMETRY_FIELDS)}"
        )
    return values


def fault_mask(values: list[str]) -> int:
    text = values[TELEMETRY_FIELDS.index("fault_mask")]
    return int(text, 0)


def telemetry_ready(values: list[str]) -> bool:
    attitude_ok = values[TELEMETRY_FIELDS.index("attitude_ok")]
    vel_valid = values[TELEMETRY_FIELDS.index("vel_valid")]
    return attitude_ok == "1" and vel_valid == "1" and fault_mask(values) == 0


def capture_until(
    port: serial.Serial,
    writer: csv.writer,
    deadline: float,
    phase: str,
    rows: list[int],
    require_ready: bool = False,
) -> bool:
    ready = False
    while time.monotonic() < deadline:
        line = read_line(port)
        if not line:
            continue
        if line.startswith("FAULT,"):
            raise RuntimeError(line)
        values = parse_telemetry(line)
        if values is None:
            continue
        mask = fault_mask(values)
        if mask != 0:
            raise RuntimeError(f"firmware fault_mask became nonzero: 0x{mask:08x}")
        writer.writerow((SCHEMA_VERSION, phase, *values))
        rows[0] += 1
        ready = ready or telemetry_ready(values)
        if require_ready and ready:
            return True
    return ready


def main() -> int:
    args = parse_args()
    if args.repeat < 1:
        raise SystemExit("--repeat must be >= 1")
    if args.pre_roll < 0.0 or args.post_roll < 0.0 or args.ready_timeout <= 0.0:
        raise SystemExit("capture durations must be non-negative and ready timeout > 0")

    output = args.output or default_output_path()
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = output.with_suffix(output.suffix + ".json")

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.05)
        port.dtr = False
        port.rts = False
    except serial.SerialException as exc:
        print(f"error: cannot open {args.port}: {exc}", file=sys.stderr)
        return 2

    rows = [0]
    started_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    completed = False
    motor_config: MotorConfig | None = None

    try:
        time.sleep(0.15)
        port.reset_input_buffer()
        write_command(port, "motor stop")
        write_command(port, "telemetry off")

        status, motor_config = ensure_motor_config(
            port,
            args.motor_config,
            args.auto_calibrate,
            args.calibration_timeout,
        )
        if int(status.get("fault_mask", "0"), 0) != 0:
            write_command(port, "fault clear")
            status = wait_for_status(port)
            if int(status.get("fault_mask", "0"), 0) != 0:
                raise RuntimeError(
                    f"firmware fault remains latched: {status.get('fault_mask')}"
                )

        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("schema_version", "phase", *TELEMETRY_FIELDS))
            write_command(port, "telemetry on")

            ready_deadline = time.monotonic() + args.ready_timeout
            if not capture_until(port, writer, ready_deadline, "ready", rows, require_ready=True):
                raise RuntimeError("attitude/wheel state did not become ready before timeout")

            if args.pre_roll > 0.0:
                capture_until(
                    port, writer, time.monotonic() + args.pre_roll, "pre", rows
                )

            for repetition in range(args.repeat):
                for index, segment in enumerate(args.segment, start=1):
                    phase = f"r{repetition + 1}_s{index}"
                    if abs(segment.vq_v) < 1.0e-9:
                        write_command(port, "motor stop")
                    else:
                        write_command(port, f"motor vq {segment.vq_v:.9g}")
                    capture_until(
                        port,
                        writer,
                        time.monotonic() + segment.duration_s,
                        phase,
                        rows,
                    )

            write_command(port, "motor stop")
            if args.post_roll > 0.0:
                capture_until(
                    port, writer, time.monotonic() + args.post_roll, "post", rows
                )
            completed = True

    except (RuntimeError, serial.SerialException, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            try:
                write_command(port, "motor stop")
                write_command(port, "telemetry off")
            except serial.SerialException:
                pass
            port.close()

        metadata = {
            "format": "triwhirl-identification-run-v1",
            "telemetry_schema_version": SCHEMA_VERSION,
            "started": started_wall,
            "completed": completed,
            "port": args.port,
            "baud": args.baud,
            "repeat": args.repeat,
            "pre_roll_s": args.pre_roll,
            "post_roll_s": args.post_roll,
            "auto_calibrate": bool(args.auto_calibrate),
            "motor_config_file": str(args.motor_config),
            "motor_config": None
            if motor_config is None
            else {
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


if __name__ == "__main__":
    raise SystemExit(main())
