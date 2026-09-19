#!/usr/bin/env python3
"""Autonomous untethered swing identification over BLE.

The reaction wheel continuously pumps the Reuleaux body toward the three legal
upright vertices.  Local identification windows are captured while the *steady
pump input itself* carries the body through +/-capture_deg.  No BLE command is
sent at the vertex, because the v3 logs showed 80-125 ms host/GATT command
latency while a fast crossing can leave the local region in 60-120 ms.

To improve excitation without disturbing a vertex crossing, the pump magnitude
alternates only when the predicted body-rate sign changes (near a rocking turn).
Thus each local window sees a measured, approximately constant firmware Vq, but
successive half-cycles use different magnitudes.  The primary CSV remains
compatible with body_active_fit.py; the complete swing trajectory is also kept.

This is identification tooling, not the final ESP32 swing-up controller.
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
        description=(
            "Autonomously rock TriWhirl and capture steady-input local A/B/C "
            "crossings for plant identification."
        )
    )
    parser.add_argument(
        "--probes", type=int, default=12,
        help="number of accepted local vertex-crossing windows to collect",
    )
    parser.add_argument(
        "--pump-v-low", type=float, default=0.40,
        help="lower coarse-pump magnitude, alternated by half-cycle [V]",
    )
    parser.add_argument(
        "--pump-v-high", type=float, default=0.55,
        help="higher coarse-pump magnitude, alternated by half-cycle [V]",
    )
    parser.add_argument(
        "--capture-deg", type=float, default=8.0,
        help="start a local window after entering this vertex-error band",
    )
    parser.add_argument(
        "--probe-exit-deg", type=float, default=12.0,
        help="end a local window after leaving this wider band",
    )
    parser.add_argument(
        "--rearm-deg", type=float, default=18.0,
        help="require leaving this band before another local capture",
    )
    parser.add_argument(
        "--probe-duration", type=float, default=0.16,
        help="maximum duration of one local crossing window [s]",
    )
    parser.add_argument(
        "--min-local-rows", type=int, default=3,
        help="minimum active telemetry rows for an accepted crossing",
    )
    parser.add_argument(
        "--max-local-vq-span", type=float, default=0.03,
        help="maximum measured Vq peak-to-peak allowed inside an accepted crossing [V]",
    )
    parser.add_argument(
        "--rate-switch", type=float, default=0.03,
        help="predicted body-rate deadband for coarse pump sign switching [rad/s]",
    )
    parser.add_argument(
        "--pump-polarity", type=int, choices=(-1, 1), default=-1,
        help="Vq sign relative to predicted body-rate sign",
    )
    parser.add_argument(
        "--pump-lead-ms", type=float, default=40.0,
        help="lead used to predict body-rate sign for host/BLE pump latency",
    )
    parser.add_argument(
        "--pump-accel-alpha", type=float, default=0.75,
        help="low-pass retention for body-acceleration estimate [0,1)",
    )
    parser.add_argument(
        "--max-duration", type=float, default=50.0,
        help="hard experiment time bound [s]",
    )
    parser.add_argument(
        "--vertex-a-deg", type=float, default=68.0,
        help="IMU-frame A-vertex naming anchor; B/C are +/-120 deg",
    )
    parser.add_argument("--imu-samples", type=int, default=500)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument(
        "--motor-config", type=Path,
        default=Path("artifacts/motor-config.json"),
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def nearest_vertex(
    theta_rad: float, centers: dict[str, float]
) -> tuple[str, float, float]:
    theta_deg = math.degrees(theta_rad)
    vertex_id = min(
        centers,
        key=lambda item: abs(angle_diff_deg(theta_deg, centers[item])),
    )
    center_deg = centers[vertex_id]
    return vertex_id, center_deg, angle_diff_deg(theta_deg, center_deg)


async def run(args: argparse.Namespace) -> int:
    if args.probes < 1:
        raise RuntimeError("--probes must be >= 1")
    if not (
        math.isfinite(args.pump_v_low)
        and math.isfinite(args.pump_v_high)
        and 0.0 < args.pump_v_low <= args.pump_v_high
    ):
        raise RuntimeError("require 0 < pump-v-low <= pump-v-high")
    if not (0.0 < args.capture_deg < args.probe_exit_deg < args.rearm_deg < 60.0):
        raise RuntimeError(
            "require 0 < capture-deg < probe-exit-deg < rearm-deg < 60"
        )
    if args.probe_duration <= 0.0 or args.max_duration <= 0.0:
        raise RuntimeError("durations must be > 0")
    if args.min_local_rows < 1:
        raise RuntimeError("--min-local-rows must be >= 1")
    if args.max_local_vq_span < 0.0:
        raise RuntimeError("--max-local-vq-span must be >= 0")
    if args.rate_switch < 0.0 or args.pump_lead_ms < 0.0:
        raise RuntimeError("rate-switch and pump-lead-ms must be >= 0")
    if not (0.0 <= args.pump_accel_alpha < 1.0):
        raise RuntimeError("--pump-accel-alpha must be in [0,1)")

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
    accepted_crossings = 0
    attempted_crossings = 0
    trials: list[dict[str, object]] = []

    controller_phase = "pump"
    pump_rate_sign = 1
    half_cycle_index = 0
    current_pump_mag = args.pump_v_high
    commanded_vq: float | None = None
    pump_accel_est = 0.0
    previous_rate: float | None = None
    previous_t_us: int | None = None
    predicted_rate = 0.0
    capture_armed = True

    current_trial = 0
    current_vertex = ""
    current_center_deg = 0.0
    current_ref_rad = 0.0
    capture_t_us: int | None = None
    active_rows = 0
    local_vq_values: list[float] = []
    max_abs_error_deg = 0.0

    recent_angles: deque[tuple[int, float]] = deque()
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

    def update_pump_prediction(rate: float, t_us: int) -> float:
        nonlocal pump_accel_est, previous_rate, previous_t_us
        if previous_rate is not None and previous_t_us is not None and t_us > previous_t_us:
            dt = (t_us - previous_t_us) * 1.0e-6
            if 0.0 < dt < 0.2:
                instantaneous = (rate - previous_rate) / dt
                a = args.pump_accel_alpha
                pump_accel_est = a * pump_accel_est + (1.0 - a) * instantaneous
        previous_rate = rate
        previous_t_us = t_us
        return rate + args.pump_lead_ms * 1.0e-3 * pump_accel_est

    def update_half_cycle(predicted: float) -> None:
        nonlocal pump_rate_sign, half_cycle_index, current_pump_mag
        if abs(predicted) < args.rate_switch:
            return
        new_sign = 1 if predicted > 0.0 else -1
        if new_sign == pump_rate_sign:
            return
        pump_rate_sign = new_sign
        half_cycle_index += 1
        current_pump_mag = (
            args.pump_v_high if half_cycle_index % 2 == 0 else args.pump_v_low
        )

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
                    + ", ".join(
                        f"{key}={value:.1f} deg" for key, value in centers.items()
                    )
                )
                print(
                    f"pump magnitudes alternate {args.pump_v_low:.3f}/{args.pump_v_high:.3f} V "
                    f"at rocking turns; polarity={args.pump_polarity:+d}, "
                    f"lead={args.pump_lead_ms:.0f} ms; target={args.probes} crossings"
                )
                print(
                    "no command is sent at a vertex; the steady measured pump Vq is the ID input"
                )

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
                        "vertex_error_deg", "pump_polarity", "pump_half_cycle",
                        "pump_command_mag_v", "pump_accel_est_rad_s2",
                        "pump_predicted_rate_rad_s", *TELEMETRY_FIELDS,
                    ))
                    await transport.send("telemetry on")

                    start_mono = time.monotonic()
                    while time.monotonic() - start_mono < args.max_duration:
                        if accepted_crossings >= args.probes:
                            completed = True
                            break

                        values, row = await next_telemetry(transport, 1.0)
                        if row is None or values is None:
                            continue

                        theta = float(row["theta_rad"])
                        rate = float(row["theta_rate_rad_s"])
                        measured_vq = float(row["vq_v"])
                        t_us = int(row["t_us"])
                        predicted_rate = update_pump_prediction(rate, t_us)
                        update_half_cycle(predicted_rate)
                        desired_vq = (
                            args.pump_polarity * pump_rate_sign * current_pump_mag
                        )
                        await set_vq(transport, desired_vq)

                        vertex_id, center_deg, error_deg = nearest_vertex(theta, centers)
                        raw_writer.writerow((
                            SCHEMA_VERSION, controller_phase, vertex_id, error_deg,
                            args.pump_polarity, half_cycle_index, current_pump_mag,
                            pump_accel_est, predicted_rate, *values,
                        ))
                        total_raw_rows += 1

                        recent_angles.append((t_us, math.degrees(theta)))
                        while recent_angles and (t_us - recent_angles[0][0]) > 2_000_000:
                            recent_angles.popleft()
                        now = time.monotonic()
                        if now - last_progress_print >= 3.0 and recent_angles:
                            angles = [item[1] for item in recent_angles]
                            span = max(angles) - min(angles)
                            print(
                                f"pump progress: span_2s={span:.1f} deg, nearest={vertex_id} "
                                f"distance={abs(error_deg):.1f} deg, body_rate={rate:+.2f}, "
                                f"wheel_rate={float(row['vel_rad_s']):+.2f} rad/s, "
                                f"Vq={measured_vq:+.2f} V"
                            )
                            last_progress_print = now

                        if abs(error_deg) >= args.rearm_deg:
                            capture_armed = True

                        if controller_phase == "pump":
                            if capture_armed and abs(error_deg) <= args.capture_deg:
                                attempted_crossings += 1
                                current_trial = attempted_crossings
                                current_vertex = vertex_id
                                current_center_deg = center_deg
                                current_ref_rad = math.radians(center_deg)
                                capture_t_us = t_us
                                active_rows = 0
                                local_vq_values = []
                                max_abs_error_deg = abs(error_deg)
                                capture_armed = False
                                controller_phase = "capture"

                                local_writer.writerow((
                                    SCHEMA_VERSION, current_trial, current_vertex,
                                    current_center_deg, "armed", current_ref_rad,
                                    measured_vq, *values,
                                ))
                                local_rows += 1
                                print(
                                    f"crossing {current_trial} CAPTURE vertex {current_vertex}: "
                                    f"error={error_deg:+.2f} deg rate={rate:+.3f} rad/s, "
                                    f"steady Vq={measured_vq:+.3f} V"
                                )
                            continue

                        selected_error_deg = angle_diff_deg(
                            math.degrees(theta), current_center_deg
                        )
                        max_abs_error_deg = max(
                            max_abs_error_deg, abs(selected_error_deg)
                        )

                        if controller_phase == "capture":
                            local_writer.writerow((
                                SCHEMA_VERSION, current_trial, current_vertex,
                                current_center_deg, "active", current_ref_rad,
                                measured_vq, *values,
                            ))
                            local_rows += 1
                            active_rows += 1
                            local_vq_values.append(measured_vq)

                            assert capture_t_us is not None
                            elapsed = (t_us - capture_t_us) * 1.0e-6
                            if (
                                elapsed >= args.probe_duration
                                or abs(selected_error_deg) >= args.probe_exit_deg
                            ):
                                if local_vq_values:
                                    vq_min = min(local_vq_values)
                                    vq_max = max(local_vq_values)
                                    vq_span = vq_max - vq_min
                                    vq_mean = sum(local_vq_values) / len(local_vq_values)
                                else:
                                    vq_min = vq_max = vq_mean = 0.0
                                    vq_span = math.inf
                                accepted = (
                                    active_rows >= args.min_local_rows
                                    and vq_span <= args.max_local_vq_span
                                )
                                trials.append({
                                    "trial": current_trial,
                                    "vertex_id": current_vertex,
                                    "vertex_center_deg": current_center_deg,
                                    "active_rows": active_rows,
                                    "measured_vq_mean_v": vq_mean,
                                    "measured_vq_min_v": vq_min,
                                    "measured_vq_max_v": vq_max,
                                    "measured_vq_span_v": vq_span,
                                    "max_abs_vertex_error_deg": max_abs_error_deg,
                                    "accepted": accepted,
                                })
                                if accepted:
                                    accepted_crossings += 1
                                print(
                                    f"crossing {current_trial} complete: vertex {current_vertex}, "
                                    f"rows={active_rows}, Vq_mean={vq_mean:+.3f} V, "
                                    f"Vq_span={vq_span:.3f} V, "
                                    f"accepted={accepted}, total={accepted_crossings}/{args.probes}"
                                )
                                controller_phase = "pump"
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
            "format": "triwhirl-auto-swing-id-run-v4",
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
            "pump_v_low": args.pump_v_low,
            "pump_v_high": args.pump_v_high,
            "pump_polarity": args.pump_polarity,
            "pump_lead_ms": args.pump_lead_ms,
            "pump_accel_alpha": args.pump_accel_alpha,
            "capture_deg": args.capture_deg,
            "probe_exit_deg": args.probe_exit_deg,
            "rearm_deg": args.rearm_deg,
            "local_window_duration_s": args.probe_duration,
            "min_local_rows": args.min_local_rows,
            "max_local_vq_span_v": args.max_local_vq_span,
            "max_duration_s": args.max_duration,
            "vertex_a_deg": args.vertex_a_deg,
            "vertex_centers_deg": centers,
            "accepted_crossings": accepted_crossings,
            "attempted_crossings": attempted_crossings,
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
            f"run ended after {accepted_crossings}/{args.probes} accepted crossings "
            f"from {attempted_crossings} vertex entries"
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
