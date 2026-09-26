#!/usr/bin/env python3
"""Acquire signed near-upright TriWhirl dynamics in a vertex-agnostic local frame.

Each trial defines its own local equilibrium from the held posture:
    theta_error = wrap(theta - theta_ref)

No A/B/C label is required. Any physical upright orientation may be used on any
trial. The absolute held angle remains in theta_ref_rad as provenance, while the
identification model is expressed only in local error coordinates.

The IMU is calibrated and attitude is initialized once before telemetry starts.
Per-trial attitude resets are intentionally avoided: they are unnecessary for a
local-error model and previously coupled command/reply traffic to the live
telemetry stream.

Motor-command synchronization is explicit. Before a nonzero Vq command, live
telemetry is paused and drained until the firmware acknowledges telemetry-off;
the motor command must then receive its own `OK motor FOC` reply before telemetry
is re-enabled. This prevents command ACK/ERR records from being silently consumed
as noise by the telemetry reader and establishes a clean freshness boundary for
the measured Vq used by identification.
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
from body_local_ble import angle_diff, discover, next_telemetry, wait_for_stable_hold


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture signed local upright dynamics without A/B/C vertex labels."
    )
    parser.add_argument("--trials", type=int, default=4)
    parser.add_argument(
        "--vq", type=float, default=0.25,
        help="default absolute Vq excitation [V] used for both signs",
    )
    parser.add_argument("--vq-positive", type=float, default=None)
    parser.add_argument("--vq-negative", type=float, default=None)
    parser.add_argument(
        "--pulse-duration", type=float, default=0.12,
        help="time to keep Vq active after release detection [s]",
    )
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
    # Balanced four-trial pattern avoids a simple +,-,+,- order correlation.
    return (1, -1, -1, 1)[(trial - 1) % 4]


def excitation_for_trial(args: argparse.Namespace, trial: int) -> float:
    sign = planned_sign(trial)
    pos = args.vq if args.vq_positive is None else args.vq_positive
    neg = args.vq if args.vq_negative is None else args.vq_negative
    return pos if sign > 0 else -neg


async def set_telemetry(
    transport: BleLineTransport,
    enabled: bool,
    timeout_s: float = 3.0,
) -> str:
    command = "telemetry on" if enabled else "telemetry off"
    prefix = "OK telemetry on" if enabled else "OK telemetry off"
    await transport.send(command)
    return await read_until_prefix(transport, prefix, timeout_s)


async def arm_input_before_release(
    transport: BleLineTransport,
    writer: csv.writer,
    trial: int,
    theta_ref: float,
    planned_vq: float,
    total_rows: list[int],
    timeout_s: float = 4.0,
) -> tuple[list[str], dict[str, str]]:
    # Establish a protocol boundary before mutating actuation. With live text
    # telemetry enabled, an OK/ERR motor reply can otherwise be consumed and
    # discarded by next_telemetry(), leaving the host unable to distinguish a
    # rejected command from stale/pre-command telemetry.
    await set_telemetry(transport, False)
    await transport.send(f"motor vq {planned_vq:.9g}")
    motor_ack = await read_until_prefix(transport, "OK motor FOC vq_v=", 3.0)
    print(f"trial {trial}: firmware accepted motor command: {motor_ack}")
    await set_telemetry(transport, True)

    deadline = time.monotonic() + timeout_s
    confirmed = 0
    last_values: list[str] | None = None
    last_row: dict[str, str] | None = None

    while time.monotonic() < deadline:
        values, row = await next_telemetry(transport, 1.0)
        if row is None or values is None:
            continue
        writer.writerow((
            SCHEMA_VERSION, trial, "armed", theta_ref, planned_vq, *values,
        ))
        total_rows[0] += 1
        last_values, last_row = values, row

        measured_vq = float(row["vq_v"])
        held = (
            abs(float(row["theta_rate_rad_s"])) <= 0.20
            and abs(angle_diff(float(row["theta_rad"]), theta_ref)) <= math.radians(1.5)
        )
        if abs(measured_vq - planned_vq) <= 0.01 and held:
            confirmed += 1
        else:
            confirmed = 0
        if confirmed >= 2:
            return last_values, last_row

    await transport.send("motor stop")
    measured = None if last_row is None else last_row.get("vq_v")
    raise RuntimeError(
        f"trial {trial}: motor command was acknowledged but fresh telemetry did not "
        f"confirm held Vq before timeout (last measured vq={measured})"
    )


async def run(args: argparse.Namespace) -> int:
    if args.trials < 2:
        raise RuntimeError("--trials must be >= 2 so both Vq signs are represented")
    amplitudes = [args.vq]
    if args.vq_positive is not None:
        amplitudes.append(args.vq_positive)
    if args.vq_negative is not None:
        amplitudes.append(args.vq_negative)
    if any((not math.isfinite(v) or v <= 0.0) for v in amplitudes):
        raise RuntimeError("Vq magnitudes must be finite and > 0")
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
    total_rows = [0]
    completed = False
    motor_config = None

    stable_span_rad = math.radians(args.stable_angle_deg)
    trigger_angle_rad = math.radians(args.trigger_angle_deg)
    max_angle_rad = math.radians(args.max_angle_deg)
    positive_vq = args.vq if args.vq_positive is None else args.vq_positive
    negative_vq = args.vq if args.vq_negative is None else args.vq_negative

    print("local upright mode: no A/B/C selection; hold any physical upright each trial")
    print(f"excitation: +{positive_vq:.3f} V / -{negative_vq:.3f} V")

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
                # One calibration + one gravity initialization is sufficient.
                # Every trial below defines its own theta_ref from a stable hold.
                await prepare_imu(transport, args.imu_samples)

                with output.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow((
                        "schema_version", "trial", "phase", "theta_ref_rad",
                        "planned_vq_v", *TELEMETRY_FIELDS,
                    ))
                    await set_telemetry(transport, True)

                    for trial in range(1, args.trials + 1):
                        planned_vq = excitation_for_trial(args, trial)
                        theta_ref, baseline = await wait_for_stable_hold(
                            transport, trial, args.stable_window,
                            args.stable_rate, stable_span_rad, args.hold_timeout,
                        )
                        print(
                            f"trial {trial}: local theta_ref={theta_ref:.6f} rad "
                            f"({math.degrees(theta_ref):.2f} deg), "
                            f"planned Vq={planned_vq:+.3f} V"
                        )

                        for values, _row in baseline:
                            writer.writerow((
                                SCHEMA_VERSION, trial, "hold", theta_ref,
                                planned_vq, *values,
                            ))
                            total_rows[0] += 1

                        print(
                            f"trial {trial}: establishing {planned_vq:+.3f} V while you hold "
                            "the upright still ..."
                        )
                        await arm_input_before_release(
                            transport, writer, trial, theta_ref,
                            planned_vq, total_rows,
                        )
                        print(
                            f"trial {trial} INPUT READY: Vq is confirmed by fresh firmware telemetry. "
                            "Release your fingers now without pushing."
                        )

                        release_deadline = time.monotonic() + args.release_timeout
                        trigger_t_us: int | None = None
                        trigger_hits = 0
                        pulse_stop_sent = False
                        local_rows = 0
                        active_rows = 0
                        max_deviation = 0.0
                        end_reason = "release_timeout"

                        while time.monotonic() < release_deadline:
                            values, row = await next_telemetry(transport, 1.0)
                            if row is None or values is None:
                                continue
                            theta = float(row["theta_rad"])
                            rate = float(row["theta_rate_rad_s"])
                            measured_vq = float(row["vq_v"])
                            deviation_signed = angle_diff(theta, theta_ref)
                            deviation = abs(deviation_signed)
                            max_deviation = max(max_deviation, deviation)

                            if trigger_t_us is None:
                                writer.writerow((
                                    SCHEMA_VERSION, trial, "armed", theta_ref,
                                    planned_vq, *values,
                                ))
                                total_rows[0] += 1
                                if (
                                    abs(rate) >= args.trigger_rate
                                    or deviation >= trigger_angle_rad
                                ):
                                    trigger_hits += 1
                                else:
                                    trigger_hits = 0
                                if trigger_hits >= 2:
                                    trigger_t_us = int(row["t_us"])
                                    print(
                                        f"trial {trial} RELEASE detected: theta_error="
                                        f"{math.degrees(deviation_signed):+.2f} deg, "
                                        f"rate={rate:+.3f} rad/s, "
                                        f"measured Vq={measured_vq:+.3f} V"
                                    )
                                continue

                            elapsed = (int(row["t_us"]) - trigger_t_us) * 1.0e-6
                            if not pulse_stop_sent and elapsed >= args.pulse_duration:
                                await transport.send("motor stop")
                                pulse_stop_sent = True

                            # Phase follows the measured firmware input, not the
                            # host command-send timestamp. This keeps transition
                            # rows honest even if the stop command takes a frame
                            # to become visible in telemetry.
                            phase = "active" if abs(measured_vq) > 1.0e-4 else "zero_vector"
                            writer.writerow((
                                SCHEMA_VERSION, trial, phase, theta_ref,
                                planned_vq, *values,
                            ))
                            total_rows[0] += 1
                            local_rows += 1
                            if phase == "active":
                                active_rows += 1

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
                            "local_rows": local_rows,
                            "active_rows": active_rows,
                            "max_abs_theta_error_deg": math.degrees(max_deviation),
                            "end_reason": end_reason,
                        })
                        print(
                            f"trial {trial} captured: {local_rows} dynamic rows, "
                            f"end={end_reason}. Hold any upright for the next trial."
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
            "format": "triwhirl-body-active-local-run-v1",
            "telemetry_schema_version": SCHEMA_VERSION,
            "transport": "ble",
            "device_name": args.name,
            "device_address": resolved_address,
            "started": started_wall,
            "completed": completed,
            "coordinate_contract": (
                "Each trial uses its held theta_ref_rad as the local equilibrium; "
                "A/B/C labels are not part of acquisition or control coordinates."
            ),
            "motor_config": None if motor_config is None else {
                "pole_pairs": motor_config.pole_pairs,
                "sensor_dir": motor_config.sensor_dir,
                "offset_rad": motor_config.offset_rad,
            },
            "vq_abs_v": args.vq,
            "vq_positive_v": positive_vq,
            "vq_negative_v": negative_vq,
            "pulse_duration_after_release_s": args.pulse_duration,
            "max_local_duration_s": args.max_local_duration,
            "max_angle_deg": args.max_angle_deg,
            "total_rows": total_rows[0],
            "trials": trial_metadata,
            "csv": str(output),
        }
        metadata_path.write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

    print(f"saved {total_rows[0]} rows -> {output}")
    print(f"saved run metadata -> {metadata_path}")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(run(args))
    except (RuntimeError, OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())