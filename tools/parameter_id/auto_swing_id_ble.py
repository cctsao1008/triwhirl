#!/usr/bin/env python3
"""Autonomous untethered swing identification over BLE.

The reaction wheel first pumps the Reuleaux body toward any of the three upright
vertices.  Crucially, pumping is *not* interrupted in a wide approach band.
Only after measured attitude actually enters the local capture window does the
host command a short signed identification probe.  The primary CSV therefore
contains local windows that were physically reached, while a companion raw CSV
keeps the complete rocking trajectory.

This is host-side identification tooling, not the final ESP32 swing-up
controller.  Measured firmware ``vq_v`` remains the regression input.
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
    parser.add_argument("--capture-deg", type=float, default=8.0,
                        help="trigger a local probe only after entering this vertex-error window")
    parser.add_argument("--probe-exit-deg", type=float, default=12.0,
                        help="end the local probe after leaving this wider vertex-error window")
    parser.add_argument("--rearm-deg", type=float, default=18.0,
                        help="require leaving this vertex-error window before another probe")
    parser.add_argument("--probe-duration", type=float, default=0.12,
                        help="maximum local probe duration [s]")
    parser.add_argument("--min-observed-probe-rows", type=int, default=2,
                        help="minimum telemetry rows with measured probe Vq before counting success")
    parser.add_argument("--rate-switch", type=float, default=0.03,
                        help="predicted body-rate deadband for coarse pump sign switching [rad/s]")
    parser.add_argument("--pump-polarity", type=int, choices=(-1, 1), default=-1,
                        help="Vq sign relative to predicted body-rate sign")
    parser.add_argument("--pump-lead-ms", type=float, default=40.0,
                        help="lead used to predict body-rate sign for host/BLE pump latency")
    parser.add_argument("--pump-accel-alpha", type=float, default=0.75,
                        help="low-pass retention for body-acceleration estimate [0,1)")
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
    return vertex_id, center_deg, angle_diff_deg(theta_deg, center_deg)


def probe_voltage(index: int, positive_v: float, negative_abs_v: float) -> float:
    # Balanced 4-shot pattern.  Index is the *attempt* index so failed probes
    # cannot pin all subsequent attempts to the first (+) sign.
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
    if not (0.0 < args.capture_deg < args.probe_exit_deg < args.rearm_deg < 60.0):
        raise RuntimeError(
            "require 0 < capture-deg < probe-exit-deg < rearm-deg < 60"
        )
    if args.probe_duration <= 0.0 or args.max_duration <= 0.0:
        raise RuntimeError("durations must be > 0")
    if args.min_observed_probe_rows < 1:
        raise RuntimeError("--min-observed-probe-rows must be >= 1")
    if args.rate_switch < 0.0:
        raise RuntimeError("--rate-switch must be >= 0")
    if args.pump_lead_ms < 0.0:
        raise RuntimeError("--pump-lead-ms must be >= 0")
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
    successful_probes = 0
    attempted_probes = 0
    trials: list[dict[str, object]] = []

    controller_phase = "pump"
    pump_rate_sign = 1
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
    current_probe_v = 0.0
    capture_t_us: int | None = None
    active_rows = 0
    observed_probe_rows = 0
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
                    f"pump={args.pump_v:.3f} V polarity={args.pump_polarity:+d} "
                    f"lead={args.pump_lead_ms:.0f} ms; "
                    f"local probes=+{args.probe_v_positive:.3f}/-{args.probe_v_negative:.3f} V; "
                    f"target={args.probes} probes"
                )
                print(
                    "pump continues uninterrupted until the body actually enters "
                    f"+/-{args.capture_deg:.1f} deg of a vertex"
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
                        predicted_rate = update_pump_prediction(rate, t_us)
                        vertex_id, center_deg, error_deg = nearest_vertex(theta, centers)

                        raw_writer.writerow((
                            SCHEMA_VERSION, controller_phase, vertex_id, error_deg,
                            args.pump_polarity, pump_accel_est, predicted_rate, *values,
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
                                f"pump progress: span_2s={span:.1f} deg, "
                                f"nearest={vertex_id} distance={abs(error_deg):.1f} deg, "
                                f"body_rate={rate:+.2f}, wheel_rate={float(row['vel_rad_s']):+.2f} rad/s"
                            )
                            last_progress_print = now

                        if controller_phase == "pump":
                            if abs(predicted_rate) >= args.rate_switch:
                                pump_rate_sign = 1 if predicted_rate > 0.0 else -1
                            desired = args.pump_polarity * pump_rate_sign * args.pump_v
                            await set_vq(transport, desired)

                            if abs(error_deg) >= args.rearm_deg:
                                capture_armed = True

                            if capture_armed and abs(error_deg) <= args.capture_deg:
                                attempted_probes += 1
                                current_trial = attempted_probes
                                current_vertex = vertex_id
                                current_center_deg = center_deg
                                current_ref_rad = math.radians(center_deg)
                                current_probe_v = probe_voltage(
                                    current_trial - 1,
                                    args.probe_v_positive,
                                    args.probe_v_negative,
                                )
                                capture_t_us = t_us
                                active_rows = 0
                                observed_probe_rows = 0
                                max_abs_error_deg = abs(error_deg)
                                capture_armed = False

                                # Preserve one pre-command sample as the kinematic anchor.
                                local_writer.writerow((
                                    SCHEMA_VERSION, current_trial, current_vertex,
                                    current_center_deg, "armed", current_ref_rad,
                                    current_probe_v, *values,
                                ))
                                local_rows += 1

                                controller_phase = "active"
                                await set_vq(transport, current_probe_v)
                                print(
                                    f"probe {current_trial} CAPTURE vertex {current_vertex}: "
                                    f"error={error_deg:+.2f} deg rate={rate:+.3f} rad/s; "
                                    f"command Vq={current_probe_v:+.3f} V"
                                )
                            continue

                        selected_error_deg = angle_diff_deg(
                            math.degrees(theta), current_center_deg
                        )
                        max_abs_error_deg = max(
                            max_abs_error_deg, abs(selected_error_deg)
                        )

                        if controller_phase == "active":
                            local_writer.writerow((
                                SCHEMA_VERSION, current_trial, current_vertex,
                                current_center_deg, "active", current_ref_rad,
                                current_probe_v, *values,
                            ))
                            local_rows += 1
                            active_rows += 1
                            if abs(measured_vq - current_probe_v) <= 0.02:
                                observed_probe_rows += 1

                            assert capture_t_us is not None
                            elapsed = (t_us - capture_t_us) * 1.0e-6
                            if (
                                elapsed >= args.probe_duration
                                or abs(selected_error_deg) >= args.probe_exit_deg
                            ):
                                accepted = (
                                    observed_probe_rows >= args.min_observed_probe_rows
                                )
                                trials.append({
                                    "trial": current_trial,
                                    "vertex_id": current_vertex,
                                    "vertex_center_deg": current_center_deg,
                                    "planned_vq_v": current_probe_v,
                                    "active_rows": active_rows,
                                    "observed_probe_rows": observed_probe_rows,
                                    "max_abs_vertex_error_deg": max_abs_error_deg,
                                    "accepted": accepted,
                                })
                                if accepted:
                                    successful_probes += 1
                                print(
                                    f"probe {current_trial} complete: vertex {current_vertex}, "
                                    f"active_rows={active_rows}, measured_probe_rows={observed_probe_rows}, "
                                    f"successful={successful_probes}/{args.probes}"
                                )

                                # Resume energy pumping directly; do not issue motor stop
                                # here because stopZeroVector() is electrical braking.
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
            "format": "triwhirl-auto-swing-id-run-v3",
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
            "pump_polarity": args.pump_polarity,
            "pump_lead_ms": args.pump_lead_ms,
            "pump_accel_alpha": args.pump_accel_alpha,
            "probe_v_positive": args.probe_v_positive,
            "probe_v_negative_abs": args.probe_v_negative,
            "capture_deg": args.capture_deg,
            "probe_exit_deg": args.probe_exit_deg,
            "rearm_deg": args.rearm_deg,
            "probe_duration_s": args.probe_duration,
            "min_observed_probe_rows": args.min_observed_probe_rows,
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
            f"run ended after {successful_probes}/{args.probes} successful probes "
            f"from {attempted_probes} actual vertex entries"
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
