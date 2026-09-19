from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Sequence

from .log import decode_main, download_main, prepare_main, start_main, stop_main
from ..ble import DEVICE_NAME


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a complete firmware-owned TWLG recording lifecycle: prepare, "
            "start, wait, stop/finalize, download, and optionally decode."
        )
    )
    parser.add_argument(
        "seconds",
        nargs="?",
        type=float,
        default=45.0,
        help="requested recording interval [s]",
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="optionally decode the downloaded TWLG to this CSV path",
    )
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--address", default=None)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument(
        "--reserve-seconds",
        type=float,
        default=2.0,
        help="extra prepared capacity for host stop-command latency [s]",
    )
    parser.add_argument("--prepare-timeout", type=float, default=90.0)
    parser.add_argument("--control-timeout", type=float, default=5.0)
    parser.add_argument("--finalize-timeout", type=float, default=20.0)
    parser.add_argument("--download-timeout", type=float, default=60.0)
    return parser


def _ble_args(args: argparse.Namespace) -> list[str]:
    result = [
        "--name",
        str(args.name),
        "--scan-timeout",
        f"{args.scan_timeout:.9g}",
    ]
    if args.address:
        result.extend(("--address", str(args.address)))
    return result


def session_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    if not math.isfinite(args.seconds) or args.seconds <= 0.0:
        print("error: seconds must be finite and > 0")
        return 2
    if not math.isfinite(args.reserve_seconds) or args.reserve_seconds < 0.0:
        print("error: --reserve-seconds must be finite and >= 0")
        return 2

    prepared_seconds = args.seconds + args.reserve_seconds
    ble = _ble_args(args)

    print(
        f"TWLG session: prepare={prepared_seconds:.3f}s, "
        f"record={args.seconds:.3f}s, output={args.output}"
    )

    rc = prepare_main(
        [
            f"{prepared_seconds:.9g}",
            *ble,
            "--wait-timeout",
            f"{args.prepare_timeout:.9g}",
        ]
    )
    if rc != 0:
        return rc

    rc = start_main([*ble, "--timeout", f"{args.control_timeout:.9g}"])
    if rc != 0:
        return rc

    interrupted = False
    try:
        deadline = time.monotonic() + args.seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.25, remaining))
    except KeyboardInterrupt:
        interrupted = True
        print("recording interrupted; finalizing the captured TWLG before exit")

    rc = stop_main([*ble, "--timeout", f"{args.finalize_timeout:.9g}"])
    if rc != 0:
        return rc

    rc = download_main(
        [
            "-o",
            str(args.output),
            *ble,
            "--timeout",
            f"{args.download_timeout:.9g}",
        ]
    )
    if rc != 0:
        return rc

    if args.csv is not None:
        rc = decode_main([str(args.output), "-o", str(args.csv)])
        if rc != 0:
            return rc

    return 130 if interrupted else 0
