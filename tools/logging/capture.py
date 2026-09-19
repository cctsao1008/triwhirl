#!/usr/bin/env python3
"""Capture TriWhirl UART telemetry into an analysis-ready CSV file."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import serial

SCHEMA_VERSION = 2
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

EXPECTED_HEADER = "telemetry_fields," + ",".join(TELEMETRY_FIELDS)


def default_output_path() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path(f"triwhirl-{stamp}.csv")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture TriWhirl telemetry from the CH340 UART."
    )
    parser.add_argument("port", help="serial port, for example COM28")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="capture duration in seconds; 0 means until Ctrl+C",
    )
    parser.add_argument(
        "--no-enable",
        action="store_true",
        help="do not send 'telemetry on' after opening the port",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="do not echo non-telemetry device output",
    )
    return parser.parse_args()


def write_command(port: serial.Serial, command: str) -> None:
    port.write((command.rstrip("\r\n") + "\r\n").encode("ascii"))
    port.flush()


def main() -> int:
    args = parse_args()
    output = args.output or default_output_path()
    output.parent.mkdir(parents=True, exist_ok=True)

    rows = 0
    malformed = 0
    header_seen = False
    started = time.monotonic()

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.25)
    except serial.SerialException as exc:
        print(f"error: cannot open {args.port}: {exc}", file=sys.stderr)
        return 2

    try:
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("schema_version",) + TELEMETRY_FIELDS)

            if not args.no_enable:
                write_command(port, "telemetry on")

            print(
                f"capturing schema v{SCHEMA_VERSION} from {args.port} "
                f"at {args.baud} baud -> {output}"
            )

            while True:
                if args.duration > 0.0 and time.monotonic() - started >= args.duration:
                    break

                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line == EXPECTED_HEADER:
                    header_seen = True
                    continue

                if line.startswith("telemetry_fields,"):
                    print(
                        f"warning: firmware telemetry header does not match schema v{SCHEMA_VERSION}",
                        file=sys.stderr,
                    )
                    if not args.quiet:
                        print(line, file=sys.stderr)
                    continue

                if not line.startswith("telemetry,"):
                    if not args.quiet:
                        print(line, file=sys.stderr)
                    continue

                values = line.split(",")[1:]
                if len(values) != len(TELEMETRY_FIELDS):
                    malformed += 1
                    print(
                        f"warning: telemetry field count {len(values)}; "
                        f"expected {len(TELEMETRY_FIELDS)}",
                        file=sys.stderr,
                    )
                    continue

                writer.writerow((SCHEMA_VERSION,) + tuple(values))
                stream.flush()
                rows += 1

    except KeyboardInterrupt:
        pass
    finally:
        if port.is_open:
            if not args.no_enable:
                try:
                    write_command(port, "telemetry off")
                except serial.SerialException:
                    pass
            port.close()

    print(
        f"saved {rows} telemetry rows to {output} "
        f"(header_seen={int(header_seen)}, malformed={malformed})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
