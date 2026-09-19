#!/usr/bin/env python3
"""Autonomous untethered swing-up identification over BLE.

The host uses measured body phase to pump the reaction wheel, pre-arms a known
signed Vq while the body is approaching one of the three upright vertices, and
records only the local near-upright probe window for plant fitting. No hand
release is required.

This remains identification tooling rather than the final production swing-up
controller. The coarse pump is allowed to run over BLE, but it compensates for
host/transport phase delay with a short body-rate prediction. Local probe Vq is
established before entering the fit window, so BLE latency is not treated as
plant input timing.

The primary CSV intentionally matches body_active_fit.py:
    schema_version,trial,vertex_id,vertex_center_deg,phase,theta_ref_rad,
    planned_vq_v,<telemetry...>

A companion *-raw.csv keeps the complete rocking trajectory and pump history.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import time
from collections import deque
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
from body_free_ble import prepare_imu
from body_local_ble import discover, next_telemetry
from vertex_geometry import angle_diff_deg, vertex_centers_deg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Autonomously rock TriWhirl and collect local A/B/C Vq identification windows."
    )
    parser.add_argument("--probes", type=int, default=12,
                        help="number of successful local probe windows to collect")
    parser.add_argument("--pump-v", type=float, default=0.50,
                        help="coarse rocking Vq magnitude [V]")
    parser.add_argument("--probe-v-positive", type=float, default=0.25,
                        help="positive local probe magnitude [V]")
    parser.add_argument("--probe-v-negative", type=float, default=0.50,
                        help="absolute negative local probe magnitude [V]")
    parser.add_argument("--approach-deg", type=float, default=25.0,
                        help="pre-arm a local probe when approaching within this angle")
    parser.add_argument("--capture-deg", type=float, default=8.0,
                        help="start local capture inside this vertex error")
    parser.add_argument("--rearm-deg", type=float, default=15.0,
                        help="return to swing pumping after leaving this vertex error")
    parser.add_argument("--probe-duration", type=float, default=0.12,
                        help="maximum active local probe duration [s]")
    parser.add_argument("--zero-tail", type=float, default=0.12,
                        help="maximum zero-vector tail retained after the probe [s]")
    parser.add_argument("--rate-switch", type=float, default=0.03,
                        help="minimum predicted |body rate| used for pump sign switching [rad/s]")
    parser.add_argument("--pump-lead-ms", type=float, default=40.0,
                        help="predict body rate this far ahead to compensate BLE/control phase delay")
    parser.add_argument("--pump-accel-alpha", type=float, default=0.75,
                        help="0..1 low-pass weight for body angular-acceleration estimate")
    parser.add_argument("--pump-polarity", type=int, choices=(-1, 1), default=-1,
                        help="Vq sign relative to predicted body-rate sign")
    parser.add_argument("--auto-flip-seconds", type=float, default=0.0,
                        help="optional pump-polarity flip interval when no probe is reached; 0 disables")
    parser.add_argument("--max-duration", type=float, default=45.0,
                        help="hard experiment time bound [s]")
    parser.add_argument("--vertex-a-deg", type=float, default=68.0,
                        help="IMU-frame A-vertex naming anchor; B/C are +/-120 deg")
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--motor-config", type=Path,
                        default=Path("artifacts/motor-config.json"))
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def nearest_vertex(theta_rad: float, centers: dict[str, float]) -> tuple[str, float, float]:
    theta_deg = math.degrees(theta_rad)
    vertex_id = min(
        centers,
        key=lambda item: abs(angle_diff_deg(theta_deg, centers[item])),
    )
    center_deg = centers[vertex_id]
    error_deg = angle_diff_deg(theta_deg, center_deg)
    return vertex_id, center_deg, error_deg


def probe_voltage(index: int, positive_v: float, negative_abs_v: float) -> float:
    sign = (1, -1, -1, 1)[index % 4]
    return positive_v if sign > 0 else -negative_abs_v


async def run(args: argparse.Namespace) -> int:
    if args.probes < 1:
        raise RuntimeError("--probes must be >= 1")
    for name, value in (
        ("pump-v", args.pump_v),
        ("probe-v-positive", args.probe_v_positive),
        ("probe-v-negative", args.probe_v_negative),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name} must be finite and > 0")
    if not (0.0 < args.capture_deg < args.rearm_deg < args.approach_deg < 60.0):
        raise RuntimeError("require 0 < capture-deg < rearm-deg < approach-deg < 60")
    if args.probe_duration <= 0.0 or args.zero_tail < 0.0 or args.max_duration <= 0.0:
        raise RuntimeError("durations must be positive (zero-tail may be zero)")
    if args.rate_switch < 0.0:
        raise RuntimeError("--rate-switch must be >= 0")
    if not math.isfinite(args.pump_lead_ms) or args.pump_lead_ms < 0.0:
        raise RuntimeError("--pump-lead-ms must be finite and >= 0")
    if not 0.0 <= args.pump_accel_alpha <= 1.0:
        raise RuntimeError("--pump-accel-alpha must be between 0 and 1")

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_output = output.with_name(output.stem + "-raw.csv")
    metadata_output = output.with_suffix(output.suffix + ".json")
    centers = vertex_centers_deg(args.vertex_a_deg)

    target = await discover(args)
    resolved_address = getattr(target, "address", None) or args.address
    started_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    completed = False
    motor_config = None
    total_raw_rows = 0
    local_rows = 0
    successful_probes = 0
    attempted_probes = 0
    trials: list[dict[str, object]] = []

    controller_phase = "pump"
    pump_rate_sign = 1
    pump_polarity = args.pump_polarity
    commanded_vq: float | None = None
    current_trial = 0
    current_vertex = ""
    current_center_deg = 0.0
    current_ref_rad = 0.0
    current_probe_v = 0.0
    confirm_count = 0
    capture_t_us: int | None = None
    stop_t_us: int | None = None
    active_rows = 0
    zero_rows = 0
    max_abs_error_deg = 0.0
    last_progress = time.monotonic()

    previous_rate: float | None = None
    previous_t_us: int | None = None
    accel_est = 0.0
    predicted_rate = 0.0
    lead_s = args.pump_lead_ms * 1.0e-3
    recent_theta_deg: deque[tuple[int, float]] = deque(maxlen=256)
    last_progress_print = 0.0

    async def set_vq(transport: BleLineTransport, value: float) -> None:
        nonlocal commanded_vq
        if commanded_vq is not None and abs(commanded_vq - value) <= 1.0e-6:
            return
        if abs(value) < 1.0e-6:
            await transport.send("motor stop")
            commanded_vq = 0.0
        else:
            await transport.send(f"motor vq {value:.9g}")
            commanded_vq = value

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
                        "autonomous swing identification requires artifacts/motor-config.json"
                    )
                await prepare_imu(transport, args.imu_samples)
                await transport.send("attitude reset")
                await asyncio.sleep(0.15)

                print(
                    "autonomous swing ID vertices: "
                    + ", ".join(f"{key}={value:.1f} deg" for key, value in centers.items())
                )
                print(
                    f"pump={args.pump_v:.3f} V polarity={pump_polarity:+d} "
                    f"lead={args.pump_lead_ms:.0f} ms; "
                    f"local probes=+{args.probe_v_positive:.3f}/-{args.probe_v_negative:.3f} V; "
                    f"target={args.probes} probes"
                )
                print("place the untethered unit on the normal high-friction mat; no hand release is needed")

                with (
                    output.open("w", newline="", encoding="utf-8") as local_stream,
                    raw_output.open("w", newline="", encoding="utf-8") as raw_stream,
                ):
                    local_writer = csv.writer(local_stream)
                    raw_writer = csv.writer(raw_stream)
                    local_writer.writerow((
                        "schema_version", "trial", "vertex_id", "vertex_center_deg",
                        "phase", "theta_ref_rad", "planned_vq_v", *TELEMETRY_FIELDS,
                    ))
                    raw_writer.writerow((
                        "schema_version", "controller_phase", "nearest_vertex",
                        "vertex_error_deg", "pump_polarity",
                        "pump_accel_est_rad_s2", "pump_predicted_rate_rad_s",
                        *TELEMETRY_FIELDS,
                    ))
                    await transport.send("telemetry on")

                    start_mono = time.monotonic()
                    while time.monotonic() - start_mono < args.max_duration:
                        if successful_probes >= args.probes:
                            completed = True
                            break

                        values, row = await next_telemetry(transport, 1.0)
                        if row is None or values is None:
                            continue

                        theta = float(row["theta_rad"])
                        rate = float(row["theta_rate_rad_s"])
                        measured_vq = float(row["vq_v"])
                        t_us = int(row["t_us"])
                        vertex_id, center_deg, error_deg = nearest_vertex(theta, centers)

                        if previous_rate is not None and previous_t_us is not None:
                            dt = (t_us - previous_t_us) * 1.0e-6
                            if 0.001 <= dt <= 0.2:
                                accel = (rate - previous_rate) / dt
                                alpha = args.pump_accel_alpha
                                accel_est = (1.0 - alpha) * accel_est + alpha * accel
                        previous_rate = rate
                        previous_t_us = t_us
                        predicted_rate = rate + lead_s * accel_est

                        theta_deg = math.degrees(theta)
                        recent_theta_deg.append((t_us, theta_deg))
                        while recent_theta_deg and (t_us - recent_theta_deg[0][0]) > 2_000_000:
                            recent_theta_deg.popleft()

                        raw_writer.writerow((
                            SCHEMA_VERSION, controller_phase, vertex_id, error_deg,
                            pump_polarity, accel_est, predicted_rate, *values,
                        ))
                        total_raw_rows += 1

                        host_now = time.monotonic()
                        if host_now - last_progress_print >= 3.0 and recent_theta_deg:
                            angles = [item[1] for item in recent_theta_deg]
                            span = max(angles) - min(angles)
                            print(
                                f"pump progress: span_2s={span:.1f} deg, nearest={vertex_id} "
                                f"distance={abs(error_deg):.1f} deg, body_rate={rate:+.2f}, "
                                f"wheel_rate={float(row['vel_rad_s']):+.2f} rad/s"
                            )
                            last_progress_print = host_now

                        if controller_phase == "pump":
                            if abs(predicted_rate) >= args.rate_switch:
                                pump_rate_sign = 1 if predicted_rate > 0.0 else -1
                            desired = pump_polarity * pump_rate_sign * args.pump_v
                            await set_vq(transport, desired)

                            moving_toward = error_deg * math.degrees(rate) < 0.0
                            if abs(error_deg) <= args.approach_deg and moving_toward:
                                attempted_probes += 1
                                current_trial = attempted_probes
                                current_vertex = vertex_id
                                current_center_deg = center_deg
                                current_ref_rad = math.radians(center_deg)
                                current_probe_v = probe_voltage(
                                    successful_probes,
                                    args.probe_v_positive,
                                    args.probe_v_negative,
                                )
                                confirm_count = 0
                                capture_t_us = None
                                stop_t_us = None
                                active_rows = 0
                                zero_rows = 0
                                max_abs_error_deg = abs(error_deg)
                                controller_phase = "armed"
                                await set_vq(transport, current_probe_v)
                                print(
                                    f"probe {current_trial}: pre-arm vertex {current_vertex} "
                                    f"error={error_deg:+.2f} deg Vq={current_probe_v:+.3f} V"
                                )
                            elif (
                                args.auto_flip_seconds > 0.0
                                and time.monotonic() - last_progress >= args.auto_flip_seconds
                            ):
                                pump_polarity *= -1
                                last_progress = time.monotonic()
                                commanded_vq = None
                                print(
                                    f"no vertex capture for {args.auto_flip_seconds:.1f} s; "
                                    f"reversing pump polarity to {pump_polarity:+d}"
                                )
                            continue

                        selected_error_deg = angle_diff_deg(
                            math.degrees(theta), current_center_deg
                        )
                        max_abs_error_deg = max(max_abs_error_deg, abs(selected_error_deg))

                        if controller_phase == "armed":
                            local_writer.writerow((
                                SCHEMA_VERSION, current_trial, current_vertex,
                                current_center_deg, "armed", current_ref_rad,
                                current_probe_v, *values,
                            ))
                            local_rows += 1

                            if abs(measured_vq - current_probe_v) <= 0.02:
                                confirm_count += 1
                            else:
                                confirm_count = 0

                            moving_toward = selected_error_deg * math.degrees(rate) < 0.0
                            if abs(selected_error_deg) > args.approach_deg and not moving_toward:
                                print(
                                    f"probe {current_trial}: missed vertex {current_vertex}; resume pumping"
                                )
                                controller_phase = "recovery"
                                await set_vq(transport, 0.0)
                                continue

                            if confirm_count >= 2 and abs(selected_error_deg) <= args.capture_deg:
                                capture_t_us = t_us
                                controller_phase = "active"
                                print(
                                    f"probe {current_trial} CAPTURE vertex {current_vertex}: "
                                    f"error={selected_error_deg:+.2f} deg rate={rate:+.3f} rad/s "
                                    f"measured Vq={measured_vq:+.3f} V"
                                )
                            continue

                        if controller_phase == "active":
                            local_writer.writerow((
                                SCHEMA_VERSION, current_trial, current_vertex,
                                current_center_deg, "active", current_ref_rad,
                                current_probe_v, *values,
                            ))
                            local_rows += 1
                            active_rows += 1
                            assert capture_t_us is not None
                            elapsed = (t_us - capture_t_us) * 1.0e-6
                            if elapsed >= args.probe_duration or abs(selected_error_deg) > args.capture_deg:
                                await set_vq(transport, 0.0)
                                stop_t_us = t_us
                                controller_phase = "zero_tail"
                            continue

                        if controller_phase == "zero_tail":
                            phase = "zero_vector" if abs(measured_vq) <= 0.01 else "active"
                            local_writer.writerow((
                                SCHEMA_VERSION, current_trial, current_vertex,
                                current_center_deg, phase, current_ref_rad,
                                current_probe_v, *values,
                            ))
                            local_rows += 1
                            if phase == "zero_vector":
                                zero_rows += 1
                            assert stop_t_us is not None
                            tail_elapsed = (t_us - stop_t_us) * 1.0e-6
                            if abs(selected_error_deg) >= args.rearm_deg or tail_elapsed >= args.zero_tail:
                                successful_probes += 1
                                last_progress = time.monotonic()
                                trials.append({
                                    "trial": current_trial,
                                    "vertex_id": current_vertex,
                                    "vertex_center_deg": current_center_deg,
                                    "planned_vq_v": current_probe_v,
                                    "active_rows": active_rows,
                                    "zero_rows": zero_rows,
                                    "max_abs_vertex_error_deg": max_abs_error_deg,
                                })
                                print(
                                    f"probe {current_trial} complete: vertex {current_vertex}, "
                                    f"active_rows={active_rows}, zero_rows={zero_rows}; "
                                    f"successful={successful_probes}/{args.probes}"
                                )
                                if successful_probes >= args.probes:
                                    completed = True
                                    break
                                controller_phase = "recovery"
                            continue

                        if controller_phase == "recovery":
                            await set_vq(transport, 0.0)
                            if abs(selected_error_deg) >= args.rearm_deg:
                                controller_phase = "pump"
                                commanded_vq = None
                            continue

                await set_vq(transport, 0.0)
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
            "format": "triwhirl-auto-swing-id-run-v2",
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
            "pump_v": args.pump_v,
            "pump_polarity_initial": args.pump_polarity,
            "pump_polarity_final": pump_polarity,
            "pump_lead_ms": args.pump_lead_ms,
            "pump_accel_alpha": args.pump_accel_alpha,
            "probe_v_positive": args.probe_v_positive,
            "probe_v_negative_abs": args.probe_v_negative,
            "approach_deg": args.approach_deg,
            "capture_deg": args.capture_deg,
            "rearm_deg": args.rearm_deg,
            "probe_duration_s": args.probe_duration,
            "zero_tail_s": args.zero_tail,
            "max_duration_s": args.max_duration,
            "vertex_a_deg": args.vertex_a_deg,
            "vertex_centers_deg": centers,
            "successful_probes": successful_probes,
            "attempted_probes": attempted_probes,
            "local_rows": local_rows,
            "raw_rows": total_raw_rows,
            "trials": trials,
            "local_csv": str(output),
            "raw_csv": str(raw_output),
        }
        metadata_output.write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

    print(f"saved local identification windows -> {output}")
    print(f"saved complete swing trajectory -> {raw_output}")
    print(f"saved run metadata -> {metadata_output}")
    if not completed:
        print(
            f"run ended after {successful_probes}/{args.probes} successful probes; "
            "the retained windows are still usable"
        )
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
