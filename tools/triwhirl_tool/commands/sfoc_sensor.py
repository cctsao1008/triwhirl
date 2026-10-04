from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


SCHEMA_VERSION = 1
PREFIX = "sfoc_sensor"
FIELDS = (
    "t_us",
    "sample",
    "valid",
    "wire_error",
    "angle_rad",
    "velocity_rad_s",
    "service_us",
    "service_min_us",
    "service_max_us",
    "service_mean_us",
    "period_min_us",
    "period_max_us",
    "period_mean_us",
    "invalid_samples",
)
CSV_FIELDS = ("schema_version", "label", *FIELDS)


@dataclass(frozen=True)
class SensorRow:
    t_us: int
    sample: int
    valid: int
    wire_error: int
    angle_rad: float
    velocity_rad_s: float
    service_us: int
    service_min_us: int
    service_max_us: int
    service_mean_us: float
    period_min_us: int
    period_max_us: int
    period_mean_us: float
    invalid_samples: int
    label: str = ""

    def as_csv_row(self) -> dict[str, object]:
        return {
            "schema_version": SCHEMA_VERSION,
            "label": self.label,
            "t_us": self.t_us,
            "sample": self.sample,
            "valid": self.valid,
            "wire_error": self.wire_error,
            "angle_rad": f"{self.angle_rad:.9g}",
            "velocity_rad_s": f"{self.velocity_rad_s:.9g}",
            "service_us": self.service_us,
            "service_min_us": self.service_min_us,
            "service_max_us": self.service_max_us,
            "service_mean_us": f"{self.service_mean_us:.9g}",
            "period_min_us": self.period_min_us,
            "period_max_us": self.period_max_us,
            "period_mean_us": f"{self.period_mean_us:.9g}",
            "invalid_samples": self.invalid_samples,
        }


def _finite_float(value: str, field: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{field} is not finite")
    return parsed


def _nonnegative_int(value: str, field: str) -> int:
    parsed = int(value, 10)
    if parsed < 0:
        raise ValueError(f"{field} is negative")
    return parsed


def parse_sensor_line(line: str, label: str = "") -> SensorRow | None:
    text = line.strip()
    if not text.startswith(PREFIX + ","):
        return None

    values: dict[str, str] = {}
    for token in text.split(",")[1:]:
        key, separator, value = token.partition("=")
        if not separator or not key or key in values:
            raise ValueError(f"malformed token: {token!r}")
        values[key] = value

    missing = [field for field in FIELDS if field not in values]
    if missing:
        raise ValueError(f"missing fields: {missing}")

    return SensorRow(
        t_us=_nonnegative_int(values["t_us"], "t_us"),
        sample=_nonnegative_int(values["sample"], "sample"),
        valid=_nonnegative_int(values["valid"], "valid"),
        wire_error=_nonnegative_int(values["wire_error"], "wire_error"),
        angle_rad=_finite_float(values["angle_rad"], "angle_rad"),
        velocity_rad_s=_finite_float(values["velocity_rad_s"], "velocity_rad_s"),
        service_us=_nonnegative_int(values["service_us"], "service_us"),
        service_min_us=_nonnegative_int(values["service_min_us"], "service_min_us"),
        service_max_us=_nonnegative_int(values["service_max_us"], "service_max_us"),
        service_mean_us=_finite_float(values["service_mean_us"], "service_mean_us"),
        period_min_us=_nonnegative_int(values["period_min_us"], "period_min_us"),
        period_max_us=_nonnegative_int(values["period_max_us"], "period_max_us"),
        period_mean_us=_finite_float(values["period_mean_us"], "period_mean_us"),
        invalid_samples=_nonnegative_int(values["invalid_samples"], "invalid_samples"),
        label=label,
    )


def _uint32_delta(current: int, previous: int) -> int:
    return (current - previous) & 0xFFFFFFFF


def unwrap_angles(values: Sequence[float]) -> list[float]:
    if not values:
        return []
    result = [values[0]]
    previous = values[0]
    current_unwrapped = values[0]
    for value in values[1:]:
        delta = value - previous
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        current_unwrapped += delta
        result.append(current_unwrapped)
        previous = value
    return result


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def _sign(value: float, epsilon: float = 1.0e-6) -> str:
    if value > epsilon:
        return "positive"
    if value < -epsilon:
        return "negative"
    return "zero"


def summarize_rows(rows: Sequence[SensorRow]) -> dict[str, object]:
    if not rows:
        raise ValueError("no SimpleFOC sensor rows")

    velocities = [row.velocity_rad_s for row in rows]
    angles = unwrap_angles([row.angle_rad for row in rows])
    services = [float(row.service_us) for row in rows]
    telemetry_periods = [
        float(_uint32_delta(current.t_us, previous.t_us))
        for previous, current in zip(rows, rows[1:])
    ]
    elapsed_us = sum(telemetry_periods)
    elapsed_s = elapsed_us / 1_000_000.0
    telemetry_hz = ((len(rows) - 1) / elapsed_s) if elapsed_s > 0.0 else 0.0
    valid_rows = sum(1 for row in rows if row.valid != 0)
    wire_error_rows = sum(1 for row in rows if row.wire_error != 0)
    velocity_mean = statistics.fmean(velocities)
    velocity_median = statistics.median(velocities)
    velocity_std = statistics.pstdev(velocities) if len(velocities) > 1 else 0.0
    velocity_rms = math.sqrt(statistics.fmean(value * value for value in velocities))
    latest = rows[-1]
    labels = sorted({row.label for row in rows if row.label})

    return {
        "rows": len(rows),
        "label": labels[0] if len(labels) == 1 else ("mixed" if labels else ""),
        "elapsed_s": elapsed_s,
        "telemetry_hz": telemetry_hz,
        "valid_rows": valid_rows,
        "invalid_rows": len(rows) - valid_rows,
        "wire_error_rows": wire_error_rows,
        "velocity_mean_rad_s": velocity_mean,
        "velocity_median_rad_s": velocity_median,
        "velocity_std_rad_s": velocity_std,
        "velocity_rms_rad_s": velocity_rms,
        "velocity_min_rad_s": min(velocities),
        "velocity_max_rad_s": max(velocities),
        "velocity_median_sign": _sign(velocity_median),
        "angle_span_rad": max(angles) - min(angles),
        "service_min_us": min(services),
        "service_median_us": statistics.median(services),
        "service_p95_us": _percentile(services, 0.95),
        "service_max_us": max(services),
        "telemetry_period_min_us": min(telemetry_periods) if telemetry_periods else 0.0,
        "telemetry_period_median_us": statistics.median(telemetry_periods) if telemetry_periods else 0.0,
        "telemetry_period_p95_us": _percentile(telemetry_periods, 0.95) if telemetry_periods else 0.0,
        "telemetry_period_max_us": max(telemetry_periods) if telemetry_periods else 0.0,
        "firmware_period_min_us": latest.period_min_us,
        "firmware_period_max_us": latest.period_max_us,
        "firmware_period_mean_us": latest.period_mean_us,
        "firmware_invalid_samples": latest.invalid_samples,
        "firmware_service_min_us": latest.service_min_us,
        "firmware_service_max_us": latest.service_max_us,
        "firmware_service_mean_us": latest.service_mean_us,
    }


def _read_csv(path: Path) -> list[SensorRow]:
    rows: list[SensorRow] = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = [field for field in CSV_FIELDS if field not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"CSV missing fields: {missing}")
        for index, item in enumerate(reader, start=2):
            try:
                version = int(item["schema_version"])
                if version != SCHEMA_VERSION:
                    raise ValueError(f"unsupported schema_version={version}")
                rows.append(
                    SensorRow(
                        t_us=_nonnegative_int(item["t_us"], "t_us"),
                        sample=_nonnegative_int(item["sample"], "sample"),
                        valid=_nonnegative_int(item["valid"], "valid"),
                        wire_error=_nonnegative_int(item["wire_error"], "wire_error"),
                        angle_rad=_finite_float(item["angle_rad"], "angle_rad"),
                        velocity_rad_s=_finite_float(item["velocity_rad_s"], "velocity_rad_s"),
                        service_us=_nonnegative_int(item["service_us"], "service_us"),
                        service_min_us=_nonnegative_int(item["service_min_us"], "service_min_us"),
                        service_max_us=_nonnegative_int(item["service_max_us"], "service_max_us"),
                        service_mean_us=_finite_float(item["service_mean_us"], "service_mean_us"),
                        period_min_us=_nonnegative_int(item["period_min_us"], "period_min_us"),
                        period_max_us=_nonnegative_int(item["period_max_us"], "period_max_us"),
                        period_mean_us=_finite_float(item["period_mean_us"], "period_mean_us"),
                        invalid_samples=_nonnegative_int(item["invalid_samples"], "invalid_samples"),
                        label=item.get("label", ""),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid CSV row {index}: {exc}") from exc
    return rows


def _format_value(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def _print_summary(summary: dict[str, object]) -> None:
    for key, value in summary.items():
        print(f"{key}={_format_value(value)}")


def _capture(args: argparse.Namespace) -> int:
    try:
        import serial  # type: ignore
    except ImportError:
        print("twtool: live capture requires pyserial (python -m pip install pyserial)", file=sys.stderr)
        return 2

    output: Path = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    rows = 0
    malformed = 0
    ignored = 0

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.25)
    except serial.SerialException as exc:
        print(f"twtool: cannot open {args.port}: {exc}", file=sys.stderr)
        return 2

    try:
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            print(
                f"capturing passive SimpleFOC sensor telemetry from {args.port} "
                f"at {args.baud} baud -> {output}"
            )
            while time.monotonic() - started < args.duration:
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                try:
                    row = parse_sensor_line(line, args.label)
                except ValueError as exc:
                    malformed += 1
                    if not args.quiet:
                        print(f"warning: {exc}: {line}", file=sys.stderr)
                    continue
                if row is None:
                    ignored += 1
                    if not args.quiet:
                        print(line, file=sys.stderr)
                    continue
                writer.writerow(row.as_csv_row())
                stream.flush()
                rows += 1
    except KeyboardInterrupt:
        pass
    finally:
        port.close()

    print(f"saved_rows={rows}")
    print(f"malformed_lines={malformed}")
    print(f"ignored_lines={ignored}")
    print(f"output={output}")
    return 0 if rows > 0 else 1


def _analyze(args: argparse.Namespace) -> int:
    try:
        rows = _read_csv(args.input)
        summary = summarize_rows(rows)
    except (OSError, ValueError) as exc:
        print(f"twtool: {exc}", file=sys.stderr)
        return 2
    _print_summary(summary)
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="twtool diag sfoc-sensor",
        description=(
            "Capture and analyze passive SimpleFOC-owned AS5600 commissioning telemetry. "
            "This tool never sends motor/runtime commands."
        ),
    )
    subparsers = parser.add_subparsers(dest="action", required=True)

    capture = subparsers.add_parser("capture", help="capture sfoc_sensor UART rows to canonical CSV")
    capture.add_argument("port", help="serial port, for example COM14")
    capture.add_argument("-b", "--baud", type=int, default=115200)
    capture.add_argument("--duration", type=float, default=20.0, help="capture duration in seconds")
    capture.add_argument("-o", "--output", type=Path, required=True)
    capture.add_argument("--label", default="", help="operator label such as stationary, cw, or ccw")
    capture.add_argument("--quiet", action="store_true", help="suppress non-sensor and malformed-line echo")

    analyze = subparsers.add_parser("analyze", help="summarize a canonical sensor commissioning CSV")
    analyze.add_argument("input", type=Path)
    return parser


def sfoc_sensor_main(argv: Sequence[str]) -> int:
    parser = _parser()
    args = parser.parse_args(list(argv))
    if args.action == "capture":
        if args.duration <= 0.0:
            parser.error("--duration must be > 0")
        return _capture(args)
    if args.action == "analyze":
        return _analyze(args)
    parser.error(f"unsupported action: {args.action}")
    return 2
