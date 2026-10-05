from __future__ import annotations

import argparse
import math
import re
import sys
import time
from typing import Any, Sequence


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="twtool diag sfoc-motor",
        description=(
            "Run one bounded Bluetooth-SPP SimpleFOC motor commissioning cycle: "
            "initFOC, +target velocity, stop, -target velocity, stop."
        ),
    )
    parser.add_argument(
        "port",
        nargs="?",
        default=None,
        help=(
            "optional Windows Bluetooth SPP COM port, for example COM32; "
            "omit to auto-discover by passive sfoc_motor_status handshake"
        ),
    )
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
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=6.0,
        help="maximum passive status-probe time per Bluetooth COM attempt [s]",
    )
    parser.add_argument(
        "--scan-attempts",
        type=int,
        default=3,
        help="bounded passive open/handshake attempts per Bluetooth COM candidate",
    )
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


def _com_sort_key(device: str) -> tuple[int, str]:
    match = re.fullmatch(r"COM(\d+)", device.upper())
    return (int(match.group(1)) if match else -1, device.upper())


def _bluetooth_candidates(port_infos: Sequence[Any]) -> list[str]:
    """Return Windows Bluetooth serial ports, newest/highest COM first.

    Do not infer the usable RFCOMM side from BTHENUM metadata. Windows may expose
    multiple serial endpoints for one pairing. The firmware protocol handshake is
    the authority; this filter exists only to avoid probing unrelated USB/UARTs.
    """
    devices = {
        str(info.device)
        for info in port_infos
        if "BTHENUM" in str(getattr(info, "hwid", "")).upper()
    }
    return sorted(devices, key=_com_sort_key, reverse=True)


def _close_quietly(port: Any) -> None:
    try:
        port.close()
    except Exception:
        pass


def _probe_status(
    serial_module: Any,
    device: str,
    baud: int,
    timeout_s: float,
    attempts: int,
) -> tuple[Any | None, str | None]:
    """Accept one candidate only after a passive motor-status identity handshake.

    Windows RFCOMM endpoints can transiently fail to open or accept I/O after a
    previous connection. Bound both read and write operations and avoid extra COM
    control operations during discovery so one stale endpoint cannot hold the scan.
    """
    retry_delay_s = 0.75
    for attempt in range(1, attempts + 1):
        try:
            port = serial_module.Serial(
                device,
                baud,
                timeout=0.25,
                write_timeout=0.5,
            )
        except (OSError, serial_module.SerialException) as exc:
            print(f"probe {device}: attempt {attempt}/{attempts} open failed: {exc}")
            if attempt < attempts:
                time.sleep(retry_delay_s)
            continue

        print(f"probe {device}: attempt {attempt}/{attempts} passive status only")
        try:
            deadline = time.monotonic() + timeout_s
            next_probe = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_probe:
                    port.write(b"status\r\n")
                    next_probe = now + 0.5
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if line.startswith("sfoc_motor_status,"):
                    return port, line
        except (OSError, serial_module.SerialException) as exc:
            print(f"probe {device}: attempt {attempt}/{attempts} transport error: {exc}")
        else:
            print(f"probe {device}: attempt {attempt}/{attempts} no motor status")

        _close_quietly(port)
        if attempt < attempts:
            time.sleep(retry_delay_s)

    return None, None


def _open_motor_port(
    serial_module: Any,
    list_ports_module: Any,
    explicit_port: str | None,
    baud: int,
    scan_timeout_s: float,
    scan_attempts: int,
) -> tuple[Any | None, str | None]:
    if explicit_port is not None:
        print(f"opening requested port {explicit_port}; proving SPP command path")
        return _probe_status(
            serial_module, explicit_port, baud, scan_timeout_s, scan_attempts
        )

    candidates = _bluetooth_candidates(list_ports_module.comports())
    if not candidates:
        print("twtool: no BTHENUM Bluetooth serial ports found", file=sys.stderr)
        return None, None

    print("auto-discovering TriWhirl motor SPP: " + ", ".join(candidates))
    for device in candidates:
        port, status_line = _probe_status(
            serial_module, device, baud, scan_timeout_s, scan_attempts
        )
        if port is not None:
            print(f"selected {device} by sfoc_motor_status handshake")
            return port, status_line

    print("twtool: no Bluetooth COM port answered sfoc_motor_status", file=sys.stderr)
    return None, None


def sfoc_motor_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        print("twtool: sfoc-motor requires pyserial (python -m pip install pyserial)", file=sys.stderr)
        return 2

    if (
        args.drive_seconds <= 0.0
        or args.timeout <= 0.0
        or args.scan_timeout <= 0.0
        or args.scan_attempts <= 0
    ):
        print("twtool: timing arguments and scan attempts must be > 0", file=sys.stderr)
        return 2
    duration_ms = int(round(args.drive_seconds * 1000.0))
    command = (
        f"commission {args.pole_pairs} {args.supply_v:.9g} {args.limit_v:.9g} "
        f"{args.align_v:.9g} {abs(args.speed):.9g} {args.p:.9g} {args.tf:.9g} "
        f"{duration_ms}\r\n"
    )

    try:
        port, status_line = _open_motor_port(
            serial,
            list_ports,
            args.port,
            args.baud,
            args.scan_timeout,
            args.scan_attempts,
        )
    except KeyboardInterrupt:
        print("SIMPLEFOC_MOTOR_COMMISSION_ABORTED during=spp_auto_discovery")
        return 130

    if port is None or status_line is None:
        print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=spp_auto_discovery")
        return 2

    print(status_line)
    print(f"opened {port.port}; SPP command path proven before motor init")
    deadline = time.monotonic() + args.timeout
    sent = False
    try:
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
                print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=backend_begin see_uart=COM28")
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

        print("SIMPLEFOC_MOTOR_COMMISSION_FAIL reason=result_timeout")
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
        _close_quietly(port)
