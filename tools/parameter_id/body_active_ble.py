#!/usr/bin/env python3
"""Acquire signed near-upright reaction-wheel excitation over BLE.

The passive upright release is dominated by one unstable eigenmode, so it cannot
separate body stiffness/damping/wheel coupling. This tool adds a short known Vq
pulse immediately after release detection while the body is still inside a
small angular window. Positive and negative pulses are alternated across trials.

The host only schedules the command; the recorded firmware telemetry `vq_v` is
the authoritative input used by the fitter.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import time
from pathlib import Path

from bleak import BleakClient

from acquire_ble import (
    DEVICE_NAME,
    SCHEMA_VERSION,
    TELEMETRY_FIELDS,
    TX_UUID,
    BleLineTransport,
    ensure_motor_config,
)
from body_free_ble import prepare_imu, read_until_prefix
from body_local_ble import (
    discover,
    next_telemetry,
    wait_for_stable_hold,
    angle_diff,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture active signed near-upright body dynamics over BLE."
    )
    parser.add_argument("--trials", type=int, default=4)
    parser.add_argument(
        "--vq", type=float, default=0.25,
        help="absolute Vq pulse amplitude [V]; default reuses the proven ±0.25 V actuator level",
    )
    parser.add_argument("--pulse-duration", type=float, default=0.10)
    parser.add_argument("--max-local-duration", type=float, default=0.35)
    parser.add_argument("--max-angle-deg", type=float, default=8.0)
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument("--stable-window", type=float, default=0.8)
    parser.add_argument("--stable-rate", type=float, default=0.10)
    parser.add_argument("--stable-angle-deg", type=float, default=1.0)
    parser.add_argument("--trigger-rate", type=float, default=0.15)
    parser.add_argument("--trigger-angle-deg", type=float, default=1.5)
    parser.add_argument("--hold-timeout", type=float, default=45.0)
    parser.add_argument("--release-timeout", type=float, default=20.0)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument(
        "--motor-config", type=Path, default=Path("artifacts/motor-config.json")
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def planned_sign(trial: int) -> int:
    # Balanced + - - + for four trials; repeats in blocks of four. The nontrivial
    # order reduces correlation with slow drift/repositioning between trials.
    return (1, -1, -1, 1)[(trial - 1) % 4]


async def run(args: argparse.Namespace) -> int:
    if args.trials < 2:
        raise RuntimeError("--trials must be >= 2 so both Vq signs are represented")
    if not math.isfinite(args.vq) or args.vq <= 0.0:
        raise RuntimeError("--vq must be finite and > 0")
    if args.pulse_duration <= 0.0 or args.max_local_duration <= args.pulse_duration:
        raise RuntimeError("require 0 < --pulse-duration < --max-local-duration")
    if args.max_angle_deg <= args.trigger_angle_deg:
        raise RuntimeError("--max-angle-deg must exceed --trigger-angle-deg")

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = output.with_suffix(output.suffix + ".json")
    target = await discover(args)
    resolved_address = getattr(target, "address", None) or args.address
    started_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    trial_metadata: list[dict[str, object]] = []
    total_rows = 0
    completed = False
    motor_config = None

    stable_span_rad = math.radians(args.stable_angle_deg)
    trigger_angle_rad = math.radians(args.trigger_angle_deg)
    max_angle_rad = math.radians(args.max_angle_deg)

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
                _status, motor_config = await ensure_motor_config(
                    transport, args.motor_config, False, 0.0
                )
                if motor_config is None:
                    raise RuntimeError(
                        "active Vq identification requires artifacts/motor-config.json"
                    )
                await prepare_imu(transport, args.imu_samples)

                with output.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow((
                        "schema_version", "trial", "phase", "theta_ref_rad",
                        "planned_vq_v", *TELEMETRY_FIELDS,
                    ))
                    await transport.send("telemetry on")

                    for trial in range(1, args.trials + 1):
                        sign = planned_sign(trial)
                        planned_vq = sign * args.vq

                        # Establish the exact held operating point after a gravity
                        # reset, using the same procedure as passive local ID.
                        await wait_for_stable_hold(
                            transport, trial, args.stable_window,
                            args.stable_rate, stable_span_rad, args.hold_timeout,
                        )
                        await transport.send("attitude reset")
                        await read_until_prefix(
                            transport, "OK attitude reset from accelerometer", 3.0
                        )
                        theta_ref, baseline = await wait_for_stable_hold(
                            transport, trial, args.stable_window,
                            args.stable_rate, stable_span_rad, args.hold_timeout,
                        )

                        print(
                            f"trial {trial}: theta_ref={theta_ref:.6f} rad "
                            f"({math.degrees(theta_ref):.2f} deg), "
                            f"planned Vq={planned_vq:+.3f} V"
                        )
                        for values, _row in baseline:
                            writer.writerow((
                                SCHEMA_VERSION, trial, "hold", theta_ref,
                                planned_vq, *values,
                            ))
                            total_rows += 1

                        print(
                            f"trial {trial} ARMED: release without pushing; "
                            "the signed Vq pulse starts automatically after release detection"
                        )

                        release_deadline = time.monotonic() + args.release_timeout
                        trigger_t_us: int | None = None
                        pulse_command_host_s: float | None = None
                        pulse_stop_sent = False
                        local_rows = 0
                        max_deviation = 0.0
                        end_reason = "release_timeout"

                        while time.monotonic() < release_deadline:
                            values, row = await next_telemetry(transport, 1.0)
                            if row is None:
                                continue
                            theta = float(row["theta_rad"])
                            rate = float(row["theta_rate_rad_s"])
                            deviation = abs(angle_diff(theta, theta_ref))
                            max_deviation = max(max_deviation, deviation)

                            if trigger_t_us is None:
                                writer.writerow((
                                    SCHEMA_VERSION, trial, "armed", theta_ref,
                                    planned_vq, *values,
                                ))
                                total_rows += 1
                                if (
                                    abs(rate) >= args.trigger_rate
                                    or deviation >= trigger_angle_rad
                                ):
                                    trigger_t_us = int(row["t_us"])
                                    pulse_command_host_s = time.monotonic()
                                    await transport.send(f"motor vq {planned_vq:.9g}")
                                    print(
                                        f"trial {trial} RELEASE: theta_error="
                                        f"{math.degrees(angle_diff(theta, theta_ref)):+.2f} deg, "
                                        f"rate={rate:+.3f} rad/s; Vq pulse commanded"
                                    )
                                continue

                            elapsed = (int(row["t_us"]) - trigger_t_us) * 1.0e-6
                            if not pulse_stop_sent and elapsed >= args.pulse_duration:
                                await transport.send("motor stop")
                                pulse_stop_sent = True

                            phase = "active" if not pulse_stop_sent else "zero_vector"
                            writer.writerow((
                                SCHEMA_VERSION, trial, phase, theta_ref,
                                planned_vq, *values,
                            ))
                            total_rows += 1
                            local_rows += 1

                            if deviation >= max_angle_rad:
                                end_reason = "angle_limit"
                                break
                            if elapsed >= args.max_local_duration:
                                end_reason = "duration_limit"
                                break

                        await transport.send("motor stop")
                        if trigger_t_us is None:
                            raise RuntimeError(
                                f"trial {trial}: no release detected before timeout"
                            )

                        trial_metadata.append({
                            "trial": trial,
                            "theta_ref_rad": theta_ref,
                            "theta_ref_deg": math.degrees(theta_ref),
                            "planned_vq_v": planned_vq,
                            "trigger_t_us": trigger_t_us,
                            "pulse_command_host_monotonic_s": pulse_command_host_s,
                            "pulse_duration_s": args.pulse_duration,
                            "local_rows": local_rows,
                            "max_abs_theta_error_deg": math.degrees(max_deviation),
                            "end_reason": end_reason,
                        })
                        print(
                            f"trial {trial} captured: {local_rows} dynamic rows, "
                            f"end={end_reason}. Reposition and hold for the next trial."
                        )

                    completed = True
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
            "format": "triwhirl-body-active-run-v1",
            "telemetry_schema_version": SCHEMA_VERSION,
            "transport": "ble",
            "device_name": args.name,
            "device_address": resolved_address,
            "started": started_wall,
            "completed": completed,
            "motor_config": None if motor_config is None else {
                "pole_pairs": motor_config.pole_pairs,
                "sensor_dir": motor_config.sensor_dir,
                "offset_rad": motor_config.offset_rad,
            },
            "vq_abs_v": args.vq,
            "pulse_duration_s": args.pulse_duration,
            "max_local_duration_s": args.max_local_duration,
            "max_angle_deg": args.max_angle_deg,
            "total_rows": total_rows,
            "trials": trial_metadata,
            "csv": str(output),
        }
        metadata_path.write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

    print(f"saved {total_rows} rows -> {output}")
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
