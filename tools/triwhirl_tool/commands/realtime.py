from __future__ import annotations

import argparse
import asyncio
import math
import time
from typing import Mapping, Sequence

from ..ble import DEVICE_NAME
from .log import (
    _close_line_transport,
    _normalize_console_line,
    _open_line_transport,
    _parse_key_values,
    _wait_console,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Profile the Core-1 loop and Core-0 sensor pipeline, then enforce "
            "the pre-Balance realtime acceptance criteria."
        )
    )
    parser.add_argument(
        "seconds",
        nargs="?",
        type=float,
        default=5.0,
        help="sensor/parallel profiling interval [s]",
    )
    parser.add_argument(
        "--baseline-seconds",
        type=float,
        default=3.0,
        help="unprofiled timing baseline interval [s]",
    )
    parser.add_argument("--max-consecutive-misses", type=int, default=1)
    parser.add_argument("--max-exec-us", type=int, default=1000)
    parser.add_argument("--max-period-us", type=int, default=1250)
    parser.add_argument("--min-completion-ratio", type=float, default=0.98)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser


def _int_field(values: Mapping[str, str], key: str) -> int:
    if key not in values:
        raise RuntimeError(f"firmware response missing {key}")
    try:
        return int(values[key], 0)
    except ValueError as exc:
        raise RuntimeError(f"invalid integer field {key}={values[key]!r}") from exc


def _classify_drdy_probe(
    profile: Mapping[str, str],
    imu: Mapping[str, str],
    profile_seconds: float,
) -> tuple[str, float, float]:
    """Classify an observation-only MPU_INT GPIO hypothesis.

    The profile-start transition resets IMU acquisition statistics and the
    encoder-side request counter used by parallel_profile. Therefore edge/request
    ratio is more robust than host wall-clock timing for identifying a 1 kHz
    DATA_RDY signal. This result is diagnostic only; it never changes realtime
    acceptance or firmware routing authority.
    """

    gpio = _int_field(imu, "drdy_gpio")
    probe_only = _int_field(imu, "drdy_probe_only")
    edges = _int_field(imu, "drdy_edges")
    requests = _int_field(profile, "requests")
    rate_hz = edges / profile_seconds if profile_seconds > 0.0 else 0.0
    edge_ratio = edges / requests if requests > 0 else 0.0

    if gpio < 0:
        return "DISABLED", rate_hz, edge_ratio
    if probe_only == 0:
        if edges == 0:
            return "VERIFIED_NO_EDGES", rate_hz, edge_ratio
        if 0.90 <= edge_ratio <= 1.10:
            return "VERIFIED_ACTIVE", rate_hz, edge_ratio
        return "VERIFIED_UNEXPECTED_RATE", rate_hz, edge_ratio
    if edges == 0:
        return "NO_EDGES", rate_hz, edge_ratio
    if 0.90 <= edge_ratio <= 1.10:
        return "MATCH", rate_hz, edge_ratio
    return "INCONCLUSIVE", rate_hz, edge_ratio


def _sensor_acquisition_count(profile: Mapping[str, str]) -> tuple[int, str]:
    """Return the encoder acquisition count used by realtime acceptance.

    `parallel_profile.completions` is the number of distinct encoder generations
    committed by Core 1. With a depth-one latest-frame mailbox, Core 1 can
    legitimately skip an intermediate published generation while still consuming
    a newer, nominally-fresh frame on the next control tick. That consumer count
    therefore is not the Core-0 encoder-generation metric documented by the
    pre-Balance gate.

    When timing-stage counts are available, use the AS5600 RAW_ANGLE read count.
    Do not fold the MPU timing-profile `sample_reads` count into this ratio:
    Mpu6050::recordSampleTiming() increments it only after a FIFO packet was
    successfully decoded. In request-driven fallback mode a request may arrive
    before the next 1-kHz FIFO packet exists, so `sample_reads` is a useful
    decoded-sample diagnostic but is not a physical I2C-attempt count. IMU and
    attitude validity remain independently gated below.
    """

    if "encoder_i2c_reads" in profile:
        return _int_field(profile, "encoder_i2c_reads"), "encoder_i2c_raw"
    return _int_field(profile, "completions"), "core1_committed"


def _evaluate_realtime_acceptance(
    profile: Mapping[str, str],
    timing: Mapping[str, str],
    status: Mapping[str, str],
    *,
    min_iterations: int,
    max_consecutive_misses: int,
    max_exec_us: int,
    max_period_us: int,
    min_completion_ratio: float,
) -> list[str]:
    failures: list[str] = []

    requests = _int_field(profile, "requests")
    sensor_acquisitions, acquisition_source = _sensor_acquisition_count(profile)
    if requests <= 0:
        failures.append("sensor pipeline produced no requests")
    elif sensor_acquisitions / requests < min_completion_ratio:
        failures.append(
            "encoder acquisition ratio "
            f"{sensor_acquisitions / requests:.4f} < {min_completion_ratio:.4f} "
            f"source={acquisition_source}"
        )

    for key in ("dispatch_failures", "read_failures", "stale_results", "join_timeouts"):
        value = _int_field(profile, key)
        if value != 0:
            failures.append(f"{key}={value}")

    misses = _int_field(profile, "max_consecutive_misses")
    if misses > max_consecutive_misses:
        failures.append(
            f"max_consecutive_misses={misses} > {max_consecutive_misses}"
        )
    if _int_field(profile, "period_ge1500") != 0:
        failures.append("profiled control period reached >=1500 us")

    target_us = _int_field(timing, "target_us")
    if target_us != 1000:
        failures.append(f"target_us={target_us}, expected 1000")
    iterations = _int_field(timing, "iterations")
    if iterations < min_iterations:
        failures.append(f"iterations={iterations} < required {min_iterations}")
    observed_max_exec = _int_field(timing, "max_exec_us")
    if observed_max_exec > max_exec_us:
        failures.append(f"max_exec_us={observed_max_exec} > {max_exec_us}")
    observed_max_period = _int_field(timing, "max_period_us")
    if observed_max_period > max_period_us:
        failures.append(f"max_period_us={observed_max_period} > {max_period_us}")
    if _int_field(timing, "overruns") != 0:
        failures.append(f"overruns={_int_field(timing, 'overruns')}")
    if _int_field(timing, "late_periods") != 0:
        failures.append(f"late_periods={_int_field(timing, 'late_periods')}")
    for key in ("uart_tx_drop_bytes", "ble_rx_drop_bytes", "ble_tx_drop_bytes"):
        value = _int_field(timing, key)
        if value != 0:
            failures.append(f"{key}={value}")

    # AS5600 MD/ML/MH/AGC/magnitude are advisory field diagnostics, not
    # realtime-control validity gates. The vendor baseline uses RAW_ANGLE and
    # shaft velocity directly without consulting those flags. Acceptance must
    # therefore follow the states actually consumed by Balance.
    required_status = {
        "sample_ok": 1,
        "vel_valid": 1,
        "imu_ok": 1,
        "attitude_ok": 1,
    }
    for key, expected in required_status.items():
        actual = _int_field(status, key)
        if actual != expected:
            failures.append(f"{key}={actual}, expected {expected}")
    if _int_field(status, "fault_mask") != 0:
        failures.append(f"fault_mask={status['fault_mask']}")

    return failures


async def _run(args: argparse.Namespace) -> int:
    if not math.isfinite(args.seconds) or args.seconds <= 0.0:
        raise RuntimeError("seconds must be finite and > 0")
    if not math.isfinite(args.baseline_seconds) or args.baseline_seconds <= 0.0:
        raise RuntimeError("baseline-seconds must be finite and > 0")
    if args.max_consecutive_misses < 0:
        raise RuntimeError("max-consecutive-misses must be >= 0")
    if args.max_exec_us <= 0 or args.max_period_us <= 0:
        raise RuntimeError("timing limits must be > 0")
    if not math.isfinite(args.min_completion_ratio) or not (
        0.0 < args.min_completion_ratio <= 1.0
    ):
        raise RuntimeError("min-completion-ratio must be in (0, 1]")

    client, transport = await _open_line_transport(args)
    try:
        await transport.send("motor stop")
        print(
            await _wait_console(
                transport,
                prefixes=("OK motor stop",),
                timeout_s=args.timeout,
            )
        )

        await transport.send("timing reset")
        print(
            await _wait_console(
                transport,
                prefixes=("OK timing reset",),
                timeout_s=args.timeout,
            )
        )
        await transport.send("timing profile on")
        print(
            await _wait_console(
                transport,
                prefixes=("OK timing profile on",),
                timeout_s=args.timeout,
            )
        )
        print(f"profiling realtime/sensor pipeline for {args.seconds:.3f} seconds...")
        await asyncio.sleep(args.seconds)
        await transport.send("timing profile off")
        print(
            await _wait_console(
                transport,
                prefixes=("OK timing profile off",),
                timeout_s=args.timeout,
            )
        )

        parallel_line: str | None = None
        profile_stage_counts: dict[str, int] = {}
        deadline = time.monotonic() + max(args.timeout, 3.0)
        saw_end = False
        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            line = await transport.read_line(min(0.5, remaining))
            if not line:
                continue
            normalized = _normalize_console_line(line)
            if "parallel_profile," in normalized:
                parallel_line = normalized[normalized.find("parallel_profile,") :]
            stage_index = normalized.find("timing_profile_stage,")
            if stage_index >= 0:
                stage_line = normalized[stage_index:]
                stage = _parse_key_values(stage_line, "timing_profile_stage")
                name = stage.get("name", "")
                if name in ("encoder_i2c_raw", "mpu_i2c"):
                    profile_stage_counts[name] = _int_field(stage, "count")
            if "timing_profile_end" in normalized:
                saw_end = True
                break
            err_index = normalized.find("ERR ")
            if err_index >= 0:
                raise RuntimeError(normalized[err_index:])
        if not saw_end:
            raise RuntimeError("timed out waiting for timing_profile_end")
        if parallel_line is None:
            parallel_line = await _wait_console(
                transport,
                prefixes=("parallel_profile,",),
                timeout_s=max(args.timeout, 3.0),
            )
        print(parallel_line)

        # Read the Core-0 acquisition counters immediately after the profiled
        # interval. resetControlProfileStats() resets these counters when timing
        # profiling starts, so the passive DRDY count belongs to the same window.
        await transport.send("imu status")
        imu_line = await _wait_console(
            transport,
            prefixes=("imu,",),
            timeout_s=args.timeout,
        )
        print(imu_line)
        profile = _parse_key_values(parallel_line, "parallel_profile")
        if "encoder_i2c_raw" in profile_stage_counts:
            profile["encoder_i2c_reads"] = str(
                profile_stage_counts["encoder_i2c_raw"]
            )
        if "mpu_i2c" in profile_stage_counts:
            profile["mpu_i2c_reads"] = str(profile_stage_counts["mpu_i2c"])
        if "encoder_i2c_reads" in profile:
            requests = _int_field(profile, "requests")
            encoder_reads = _int_field(profile, "encoder_i2c_reads")
            encoder_ratio = encoder_reads / requests if requests > 0 else 0.0
            fields = [
                "sensor_acquisition",
                f"requests={requests}",
                f"encoder_reads={encoder_reads}",
                f"ratio={encoder_ratio:.4f}",
                f"core1_committed={_int_field(profile, 'completions')}",
            ]
            if "mpu_i2c_reads" in profile:
                mpu_reads = _int_field(profile, "mpu_i2c_reads")
                mpu_decode_ratio = mpu_reads / requests if requests > 0 else 0.0
                fields.insert(3, f"imu_decoded_samples={mpu_reads}")
                fields.insert(4, f"imu_decode_ratio={mpu_decode_ratio:.4f}")
            print(",".join(fields))
        imu = _parse_key_values(imu_line, "imu")
        drdy_class, drdy_rate_hz, drdy_edge_ratio = _classify_drdy_probe(
            profile, imu, args.seconds
        )
        print(
            "drdy_probe,"
            f"gpio={_int_field(imu, 'drdy_gpio')},"
            f"probe_only={_int_field(imu, 'drdy_probe_only')},"
            f"edges={_int_field(imu, 'drdy_edges')},"
            f"rate_hz={drdy_rate_hz:.3f},"
            f"edge_request_ratio={drdy_edge_ratio:.4f},"
            f"classification={drdy_class}"
        )

        await transport.send("timing reset")
        await _wait_console(
            transport,
            prefixes=("OK timing reset",),
            timeout_s=args.timeout,
        )
        print(f"measuring clean baseline for {args.baseline_seconds:.3f} seconds...")
        await asyncio.sleep(args.baseline_seconds)
        await transport.send("timing status")
        timing_line = await _wait_console(
            transport,
            prefixes=("timing,",),
            timeout_s=args.timeout,
        )
        print(timing_line)

        await transport.send("status")
        status_line = await _wait_console(
            transport,
            prefixes=("status,",),
            timeout_s=args.timeout,
        )
        print(status_line)

        timing = _parse_key_values(timing_line, "timing")
        status = _parse_key_values(status_line, "status")
        min_iterations = max(1, int(args.baseline_seconds * 500.0))
        failures = _evaluate_realtime_acceptance(
            profile,
            timing,
            status,
            min_iterations=min_iterations,
            max_consecutive_misses=args.max_consecutive_misses,
            max_exec_us=args.max_exec_us,
            max_period_us=args.max_period_us,
            min_completion_ratio=args.min_completion_ratio,
        )
        if failures:
            for failure in failures:
                print(f"FAIL {failure}")
            print("REALTIME_ACCEPTANCE_FAIL")
            return 2
        print("REALTIME_ACCEPTANCE_PASS")
        return 0
    finally:
        await _close_line_transport(client, transport)


def realtime_check_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    try:
        return asyncio.run(_run(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
