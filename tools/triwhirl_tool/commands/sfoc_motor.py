from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Sequence


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="twtool diag sfoc-motor",
        description=(
            "Run one bounded Bluetooth-SPP SimpleFOC motor commissioning cycle: "
            "initFOC, +target velocity, stop, -target velocity, stop."
        ),
    )
    parser.add_argument("port", help="Windows Bluetooth SPP COM port, for example COM31")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("--pole-pairs", type=int, default=7)
    parser.add_argument("--supply-v", type=float, default=8.3)
    parser.add_argument("--limit-v", type=float, default=0.5)
    parser.add_argument("--align-v", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=5.0, help="absolute target velocity [rad/s]")
    parser.add_argument("--p", type=float, default=0.1, help="provisional velocity P gain")
    parser.add_argument("--tf", type=float, default=0.02, help="provisional SimpleFOC velocity LPF Tf [s]")
    parser.add_argument("--drive-seconds", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser


def _parse_result(line: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for token in line.strip().split(",")[1:]:
        key, sep, value = token.partition("=")
        if sep:
            values[key] = value
    return values


def _float(values: dict[str, str], key: str) -> float:
    value = float(values[key])
    if not math.isfinite(value):
        raise ValueError(f"{key} is not finite")
    return value


def sfoc_motor_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    try:
        import serial  # type: ignore
    except ImportError:
        print("twtool: sfoc-motor requires pyserial (python -m pip install pyserial)", file=sys.stderr)
        return 2

    if args.drive_seconds <= 0.0 or args.timeout <= 0.0:
        print("twtool: timing arguments must be > 0", file=sys.stderr)
        return 2
    duration_ms = int(round(args.drive_seconds * 1000.0))
    command = (
        f"commission {args.pole_pairs} {args.supply_v:.9g} {args.limit_v:.9g} "
        f"{args.align_v:.9g} {abs(args.speed):.9g} {args.p:.9g} {args.tf:.9g} "
        f"{duration_ms}\r\n"
    )

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.25)
    except serial.SerialException as exc:
        print(f"twtool: cannot open {args.port}: {exc}", file=sys.stderr)
        return 2

    print(f"opened {args.port}; waiting for SPP link, then starting bounded SimpleFOC motor commissioning")
    deadline = time.monotonic() + args.timeout
    sent = False
    try:
        # Windows may need several seconds to establish RFCOMM after opening the COM port.
        time.sleep(1.0)
        port.reset_input_buffer()
        port.write(command.encode("ascii"))
        port.flush()
        sent = True
        print(command.strip())

        while time.monotonic() < deadline:
            raw = port.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            print(line)
            if line.startswith("ERR sfoc_motor"):
                return 2
            if not line.startswith("sfoc_motor_result,"):
                continue

            values = _parse_result(line)
            try:
                init_ok = values.get("init_ok") == "1"
                aborted = values.get("aborted") == "1"
                faulted = values.get("backend_faulted") == "1"
                sensor_valid = values.get("sensor_valid") == "1"
                pos = _float(values, "pos_mean_rad_s")
                neg = _float(values, "neg_mean_rad_s")
            except (KeyError, ValueError) as exc:
                print(f"SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=malformed_result detail={exc}")
                return 2

            if not init_ok:
                print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=initFOC")
                return 2
            if aborted:
                print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=aborted")
                return 2
            if faulted or not sensor_valid:
                print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=backend_or_sensor_fault")
                return 2
            if pos <= 0.0 or neg >= 0.0:
                print(
                    "SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=direction "
                    f"pos_mean={pos:.6f} neg_mean={neg:.6f}"
                )
                return 2

            print(
                "SIMPLEFOC_MOTOR_COMMISSION_PASS "
                f"pos_mean={pos:.6f}rad/s neg_mean={neg:.6f}rad/s"
            )
            return 0

        print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=timeout")
        return 2
    except KeyboardInterrupt:
        if sent:
            try:
                port.write(b"stop\r\n")
                port.flush()
            except Exception:
                pass
        print("SIMPLEFOC_MOTOR_COMMISSION_ABORTED")
        return 130
    finally:
        try:
            port.write(b"stop\r\n")
            port.flush()
        except Exception:
            pass
        port.close()
