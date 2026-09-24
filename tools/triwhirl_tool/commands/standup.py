from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from pathlib import Path
from typing import Any, Sequence

from ..ble import DEVICE_NAME, TRACE_UUID
from ..host_log import host_print as print
from ..host_log import print_session_header
from ..standup_plot import plot_standup_trace
from ..standup_trace import StandupTraceCapture, save_trace, trace_end_seen
from .log import (
    _close_line_transport,
    _normalize_console_line,
    _open_line_transport,
    _parse_key_values,
    _wait_console,
)

CONTROL_RATE_HZ = 1000
TRACE_DECIMATION = 10
TRACE_RATE_HZ = CONTROL_RATE_HZ // TRACE_DECIMATION


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the firmware-owned TRC-V1.1 vendor-aligned autonomous "
            "swing-up -> balance controller while capturing its decimated "
            "binary control trace."
        )
    )
    parser.add_argument(
        "--motor-config", type=Path, default=Path("artifacts/motor-config.json")
    )
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="bounded trial duration in seconds; 0 leaves standup active",
    )
    parser.add_argument(
        "--poll-period",
        type=float,
        default=1.0,
        help="supervisory balance-status polling period; realtime data uses trace BLE",
    )
    parser.add_argument(
        "--trace-dir",
        type=Path,
        default=Path("artifacts/standup"),
        help="directory for .twtrace/.csv/.json files written after the trial",
    )
    parser.add_argument(
        "--trace-flush-timeout",
        type=float,
        default=2.0,
        help="seconds to wait for the Core-0 trace worker to flush after balance stop",
    )
    parser.add_argument(
        "--no-trace",
        action="store_true",
        help="disable the dedicated binary trace subscription",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="write a four-panel PNG after the trace has been saved",
    )
    parser.add_argument(
        "--show-plot",
        action="store_true",
        help="show the generated plot interactively; implies --plot",
    )
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser


def _load_motor_config(path: Path) -> tuple[int, int, float]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
        pole_pairs = int(payload["pole_pairs"])
        sensor_dir = int(payload["sensor_dir"])
        offset_rad = float(payload["offset_rad"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load motor config {path}: {exc}") from exc
    if not 1 <= pole_pairs <= 64:
        raise RuntimeError("motor pole_pairs must be in [1, 64]")
    if sensor_dir not in (-1, 1):
        raise RuntimeError("motor sensor_dir must be -1 or 1")
    if not math.isfinite(offset_rad):
        raise RuntimeError("motor offset_rad must be finite")
    return pole_pairs, sensor_dir, offset_rad


def _validate(args: argparse.Namespace) -> None:
    if not 50 <= args.imu_samples <= 5000:
        raise RuntimeError("--imu-samples must be in [50, 5000]")
    if not math.isfinite(args.duration) or args.duration < 0.0:
        raise RuntimeError("--duration must be finite and >= 0")
    if not math.isfinite(args.poll_period) or args.poll_period <= 0.0:
        raise RuntimeError("--poll-period must be finite and > 0")
    if not math.isfinite(args.trace_flush_timeout) or args.trace_flush_timeout < 0.0:
        raise RuntimeError("--trace-flush-timeout must be finite and >= 0")
    if args.no_trace and (args.plot or args.show_plot):
        raise RuntimeError("--plot/--show-plot require trace capture; remove --no-trace")


async def _fault_status(transport, timeout: float) -> tuple[str, dict[str, str]]:
    await transport.send("fault status")
    line = await _wait_console(transport, prefixes=("fault,",), timeout_s=timeout)
    return line, _parse_key_values(line, "fault")


async def _balance_status(transport, timeout: float) -> tuple[str, dict[str, str]]:
    await transport.send("balance status")
    line = await _wait_console(transport, prefixes=("balance,",), timeout_s=timeout)
    return line, _parse_key_values(line, "balance")


async def _start_with_transient_retry(transport, timeout: float) -> str:
    last_error: RuntimeError | None = None
    for attempt in range(4):
        await transport.send("balance start")
        try:
            return await _wait_console(
                transport,
                prefixes=("OK balance start",),
                timeout_s=timeout,
            )
        except RuntimeError as exc:
            last_error = exc
            if "ERR balance start rejected reason=imu" not in str(exc) or attempt == 3:
                raise
            await asyncio.sleep(0.08)
    assert last_error is not None
    raise last_error


async def _wait_trace_flush(capture: StandupTraceCapture, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if trace_end_seen(capture.raw):
            return True
        await asyncio.sleep(0.02)
    return trace_end_seen(capture.raw)


def _trace_prefix(trace_dir: Path) -> Path:
    return trace_dir / time.strftime("standup-%Y%m%d-%H%M%S")


def _save_trace_report(
    capture: StandupTraceCapture,
    prefix: Path,
    *,
    run_metadata: dict[str, Any],
    plot: bool = False,
    show_plot: bool = False,
) -> None:
    raw_path, csv_path, json_path, summary = save_trace(
        capture, prefix, run_metadata=run_metadata
    )
    dt_min = summary["trace_dt_min_us"]
    dt_max = summary["trace_dt_max_us"]
    dt_mean = summary["trace_dt_mean_us"]
    print(
        "standup_trace,"
        f"acceptance={'PASS' if summary['trace_acceptance_pass'] else 'FAIL'},"
        f"records={summary['records']},"
        f"trace_rate_hz={summary['trace_sample_rate_hz']:.3f},"
        f"missing={summary['missing_samples']},"
        f"ring_drops={summary['ring_dropped_records']},"
        f"transport_drops={summary['transport_dropped_bytes']},"
        f"crc_errors={summary['crc_errors']},"
        f"firmware={summary['firmware_git_head']},"
        f"dirty={1 if summary['firmware_dirty'] else 0},"
        f"host_match={1 if summary['host_matches_firmware'] else 0},"
        f"dt_min_us={dt_min if dt_min is not None else 'na'},"
        f"dt_mean_us={dt_mean if dt_mean is not None else 'na'},"
        f"dt_max_us={dt_max if dt_max is not None else 'na'},"
        f"max_notify_gap_ms={summary['max_notification_gap_ms']:.3f},"
        f"throughput_kB_s={summary['receive_throughput_kB_s']:.3f}"
    )
    print(f"standup_trace_raw={raw_path}")
    print(f"standup_trace_csv={csv_path}")
    print(f"standup_trace_meta={json_path}")
    if not summary["trace_lossless"]:
        print("WARN standup trace is not lossless; inspect the JSON metadata before tuning")
    if not summary["provenance_ok"]:
        print("WARN firmware provenance is unavailable or dirty; rebuild from a clean git commit")
    elif not summary["host_matches_firmware"]:
        print("WARN host checkout and flashed firmware commits differ")

    if plot or show_plot:
        try:
            plot_path = plot_standup_trace(
                raw_path,
                prefix.with_suffix(".png"),
                show=show_plot,
                title=f"TriWhirl standup trace — {prefix.name}",
            )
            print(f"standup_plot={plot_path}")
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"WARN standup plot not generated: {exc}")


async def _run(args: argparse.Namespace) -> int:
    pole_pairs, sensor_dir, offset_rad = _load_motor_config(args.motor_config)
    run_metadata: dict[str, Any] = {
        "requested_duration_s": args.duration,
        "imu_calibration_samples": args.imu_samples,
        "upright_reference_deg": 68.0,
        "control_rate_hz_nominal": CONTROL_RATE_HZ,
        "trace_decimation": TRACE_DECIMATION,
        "trace_rate_hz_nominal": TRACE_RATE_HZ,
        "motor_config": {
            "pole_pairs": pole_pairs,
            "sensor_dir": sensor_dir,
            "offset_rad": offset_rad,
        },
    }
    client, transport = await _open_line_transport(args)
    started = False
    trace_capture: StandupTraceCapture | None = None
    trace_prefix: Path | None = None
    trace_saved = False
    try:
        if not args.no_trace:
            trace_capture = StandupTraceCapture()
            trace_prefix = _trace_prefix(args.trace_dir)
            try:
                await client.start_notify(TRACE_UUID, trace_capture.on_notify)
            except Exception as exc:
                raise RuntimeError(
                    "standup trace characteristic unavailable; build/flash the latest firmware"
                ) from exc
            await asyncio.sleep(0.05)
            print(
                f"standup trace armed: {TRACE_RATE_HZ} Hz from 1 kHz control -> host RAM"
            )

        await transport.send("motor stop")
        print(await _wait_console(transport, prefixes=("OK motor stop",), timeout_s=args.timeout))

        fault_line, fault = await _fault_status(transport, args.timeout)
        print(fault_line)
        if fault.get("latched") == "1":
            await transport.send("fault clear")
            print(
                await _wait_console(
                    transport,
                    prefixes=("OK fault clear", "OK fault already clear"),
                    timeout_s=args.timeout,
                )
            )

        await transport.send(f"motor config {pole_pairs} {sensor_dir} {offset_rad:.9g}")
        print(await _wait_console(transport, prefixes=("OK motor config",), timeout_s=args.timeout))

        print("keep the unit stationary in its normal resting orientation for gyro calibration")
        await transport.send(f"imu calibrate {args.imu_samples}")
        print(
            await _wait_console(
                transport,
                prefixes=("OK imu gyro calibration started",),
                timeout_s=args.timeout,
            )
        )
        print(
            await _wait_console(
                transport,
                prefixes=("OK imu gyro calibration bx=",),
                timeout_s=max(10.0, args.imu_samples * 0.004),
            )
        )

        await transport.send("attitude reset")
        print(await _wait_console(transport, prefixes=("OK attitude reset",), timeout_s=args.timeout))

        print("starting vendor-aligned autonomous standup (upright reference 68 deg)")
        print(await _start_with_transient_retry(transport, args.timeout))
        started = True

        if args.duration == 0.0:
            print("STANDUP_ACTIVE")
            started = False
            return 0

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            await asyncio.sleep(min(args.poll_period, max(0.0, deadline - time.monotonic())))
            balance_line, balance = await _balance_status(transport, args.timeout)
            print(_normalize_console_line(balance_line))
            if balance.get("active") != "1":
                fault_line, _fault = await _fault_status(transport, args.timeout)
                raise RuntimeError(f"standup became inactive: {balance_line}; {fault_line}")

        await transport.send("balance stop")
        print(await _wait_console(transport, prefixes=("OK balance stop",), timeout_s=args.timeout))
        started = False

        if trace_capture is not None and trace_prefix is not None:
            flushed = await _wait_trace_flush(trace_capture, args.trace_flush_timeout)
            if not flushed:
                print("WARN standup trace end marker not received before flush timeout")
            try:
                await client.stop_notify(TRACE_UUID)
            except Exception:
                pass
            _save_trace_report(
                trace_capture,
                trace_prefix,
                run_metadata=run_metadata,
                plot=args.plot or args.show_plot,
                show_plot=args.show_plot,
            )
            trace_saved = True

        print("STANDUP_TRIAL_COMPLETE")
        return 0
    finally:
        if started and client.is_connected:
            try:
                await transport.send("balance stop")
                await _wait_console(
                    transport,
                    prefixes=("OK balance stop",),
                    timeout_s=min(args.timeout, 1.0),
                )
                started = False
            except Exception:
                pass
        if trace_capture is not None and trace_prefix is not None and not trace_saved:
            if client.is_connected:
                try:
                    await _wait_trace_flush(trace_capture, min(args.trace_flush_timeout, 0.5))
                    await client.stop_notify(TRACE_UUID)
                except Exception:
                    pass
            if trace_capture.raw:
                _save_trace_report(
                    trace_capture,
                    trace_prefix,
                    run_metadata=run_metadata,
                    plot=args.plot or args.show_plot,
                    show_plot=args.show_plot,
                )
        await _close_line_transport(client, transport)


def standup_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    print_session_header()
    try:
        _validate(args)
        return asyncio.run(_run(args))
    except KeyboardInterrupt:
        print("interrupted; standup stop requested")
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
