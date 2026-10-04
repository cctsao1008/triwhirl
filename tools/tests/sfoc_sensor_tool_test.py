#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import tempfile
from pathlib import Path

from tools.triwhirl_tool.commands.sfoc_sensor import (
    CSV_FIELDS,
    SCHEMA_VERSION,
    SensorRow,
    _read_csv,
    parse_sensor_line,
    summarize_rows,
    unwrap_angles,
)


def make_line(
    *,
    t_us: int = 1000,
    sample: int = 1,
    valid: int = 1,
    wire_error: int = 0,
    angle_rad: float = 1.0,
    velocity_rad_s: float = 0.0,
    service_us: int = 100,
    service_min_us: int = 90,
    service_max_us: int = 120,
    service_mean_us: float = 101.0,
    period_min_us: int = 990,
    period_max_us: int = 1010,
    period_mean_us: float = 1000.0,
    invalid_samples: int = 0,
) -> str:
    return (
        "sfoc_sensor,"
        f"t_us={t_us},sample={sample},valid={valid},wire_error={wire_error},"
        f"angle_rad={angle_rad},velocity_rad_s={velocity_rad_s},"
        f"service_us={service_us},service_min_us={service_min_us},"
        f"service_max_us={service_max_us},service_mean_us={service_mean_us},"
        f"period_min_us={period_min_us},period_max_us={period_max_us},"
        f"period_mean_us={period_mean_us},invalid_samples={invalid_samples}"
    )


def test_parser() -> None:
    assert parse_sensor_line("boot banner") is None
    row = parse_sensor_line(make_line(velocity_rad_s=-2.5), "cw")
    assert row is not None
    assert row.label == "cw"
    assert row.velocity_rad_s == -2.5
    assert row.valid == 1

    try:
        parse_sensor_line("sfoc_sensor,t_us=1")
    except ValueError as exc:
        assert "missing fields" in str(exc)
    else:
        raise AssertionError("missing fields must be rejected")

    try:
        parse_sensor_line(make_line().replace("wire_error=0", "wire_error=nan"))
    except ValueError:
        pass
    else:
        raise AssertionError("non-integer wire_error must be rejected")


def test_wrap_aware_unwrap() -> None:
    values = [6.20, 6.27, 0.03, 0.10]
    unwrapped = unwrap_angles(values)
    assert len(unwrapped) == 4
    assert all(unwrapped[index] > unwrapped[index - 1] for index in range(1, 4))
    assert math.isclose(unwrapped[-1] - unwrapped[0], 0.18318530717958623, rel_tol=0.0, abs_tol=1e-9)


def test_summary_and_uint32_time_wrap() -> None:
    rows = [
        SensorRow(0xFFFFFF00, 1, 1, 0, 6.20, 1.0, 90, 80, 110, 91.0, 990, 1010, 1000.0, 0, "cw"),
        SensorRow(49744, 2, 1, 0, 6.27, 2.0, 100, 80, 120, 92.0, 990, 1010, 1000.0, 0, "cw"),
        SensorRow(99744, 3, 0, 5, 0.03, 3.0, 110, 80, 130, 93.0, 990, 1010, 1000.0, 1, "cw"),
    ]
    summary = summarize_rows(rows)
    assert summary["rows"] == 3
    assert summary["label"] == "cw"
    assert math.isclose(float(summary["elapsed_s"]), 0.1, abs_tol=1e-12)
    assert math.isclose(float(summary["telemetry_hz"]), 20.0, abs_tol=1e-12)
    assert summary["valid_rows"] == 2
    assert summary["invalid_rows"] == 1
    assert summary["wire_error_rows"] == 1
    assert summary["velocity_median_sign"] == "positive"
    assert float(summary["angle_span_rad"]) > 0.1
    assert summary["firmware_invalid_samples"] == 1


def test_csv_round_trip() -> None:
    original = SensorRow(1234, 7, 1, 0, 1.25, -4.5, 88, 80, 100, 87.5, 990, 1011, 1000.2, 0, "stationary")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "capture.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerow(original.as_csv_row())
        loaded = _read_csv(path)
    assert len(loaded) == 1
    assert loaded[0].label == "stationary"
    assert loaded[0].sample == 7
    assert loaded[0].velocity_rad_s == -4.5
    assert loaded[0].t_us == 1234
    assert SCHEMA_VERSION == 1


def main() -> int:
    test_parser()
    test_wrap_aware_unwrap()
    test_summary_and_uint32_time_wrap()
    test_csv_round_trip()
    print("SimpleFOC sensor host tool: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
