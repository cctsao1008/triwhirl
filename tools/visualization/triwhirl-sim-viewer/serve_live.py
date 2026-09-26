#!/usr/bin/env python3
"""Serve the TriWhirl simulation console and fresh native SITL evidence over SSE.

The browser is display-only. Every Run request launches the native C++ SITL,
which compiles the production StandupController and performs the plant
integration. Python only parses/downsamples that evidence and transports it to
the browser; no controller or plant dynamics are reimplemented here.
"""

from __future__ import annotations

import argparse
import csv
import errno
import json
import math
import os
import subprocess
import tempfile
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

CLIENT_DISCONNECT_ERRORS = (
    BrokenPipeError,
    ConnectionResetError,
    ConnectionAbortedError,
)
CLIENT_DISCONNECT_ERRNOS = {errno.EPIPE, errno.ECONNRESET, errno.ECONNABORTED}
CLIENT_DISCONNECT_WINERRORS = {10053, 10054, 10058}

NUMERIC_FIELDS = {
    "t_s",
    "true_error_rad",
    "true_error_deg",
    "true_theta_rate_rad_s",
    "true_wheel_rate_rad_s",
    "true_wheel_angle_rad",
    "filtered_rate_rad_s",
    "target_velocity_rad_s",
    "velocity_error_rad_s",
    "velocity_integral_v",
    "vq_unclamped_v",
    "vq_target_v",
    "vq_applied_v",
}
BOOL_FIELDS = {"settling", "stable", "target_saturated", "vq_saturated"}


def is_client_disconnect(error: BaseException) -> bool:
    if isinstance(error, CLIENT_DISCONNECT_ERRORS):
        return True
    if isinstance(error, OSError):
        if error.errno in CLIENT_DISCONNECT_ERRNOS:
            return True
        if getattr(error, "winerror", None) in CLIENT_DISCONNECT_WINERRORS:
            return True
    return False


def sse_payload(event: str, payload: object) -> bytes:
    data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    return f"event: {event}\ndata: {data}\n\n".encode("utf-8")


def default_sitl_candidates() -> list[Path]:
    exe = "triwhirl-standup-sitl.exe" if os.name == "nt" else "triwhirl-standup-sitl"
    return [
        ROOT / "build" / "sitl" / "Release" / exe,
        ROOT / "build" / "sitl" / "Debug" / exe,
        ROOT / "build" / "sitl" / exe,
    ]


def resolve_sitl_executable(explicit: str | None) -> Path:
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        if candidate.is_file():
            return candidate
        raise FileNotFoundError(f"SITL executable not found: {candidate}")
    for candidate in default_sitl_candidates():
        if candidate.is_file():
            return candidate
    searched = "\n  ".join(str(path) for path in default_sitl_candidates())
    raise FileNotFoundError(
        "TriWhirl SITL executable not found. Build it first. Searched:\n  " + searched
    )


def parse_row(row: dict[str, str]) -> dict[str, object]:
    sample: dict[str, object] = {}
    for key, value in row.items():
        if key in NUMERIC_FIELDS:
            sample[key] = float(value)
        elif key in BOOL_FIELDS:
            sample[key] = value == "1"
        else:
            sample[key] = value
    return sample


def run_fresh_simulation(executable: Path, profile: str) -> tuple[list[dict[str, object]], bool, str]:
    temp = tempfile.NamedTemporaryFile(prefix="triwhirl-sitl-", suffix=".csv", delete=False)
    temp_path = Path(temp.name)
    temp.close()
    try:
        command = [
            str(executable),
            "--scenario",
            "balance-demo",
            "--profile",
            profile,
            "--duration-ms",
            "10000",
            "--output",
            str(temp_path),
            "--require-balance-gate",
        ]
        completed = subprocess.run(
            command,
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
        stdout = completed.stdout.strip()
        stderr = completed.stderr.strip()
        if completed.returncode not in (0, 1):
            raise RuntimeError(stderr or stdout or f"native SITL exited with {completed.returncode}")
        if not temp_path.is_file() or temp_path.stat().st_size == 0:
            raise RuntimeError(stderr or stdout or "native SITL produced no evidence CSV")
        with temp_path.open(newline="", encoding="utf-8") as stream:
            samples = [parse_row(row) for row in csv.DictReader(stream)]
        if not samples:
            raise RuntimeError("native SITL evidence CSV contained no samples")
        gate_pass = completed.returncode == 0 and "balance_gate=PASS" in stdout
        summary = stdout.splitlines()[-1] if stdout else (stderr or "native SITL finished")
        return samples, gate_pass, summary
    finally:
        try:
            temp_path.unlink()
        except FileNotFoundError:
            pass


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    sitl_executable: Path

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        print(f"[{self.log_date_time_string()}] {fmt % args}")

    def handle(self) -> None:
        try:
            super().handle()
        except OSError as error:
            if not is_client_disconnect(error):
                raise

    def finish(self) -> None:
        try:
            super().finish()
        except OSError as error:
            if not is_client_disconnect(error):
                raise

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            body = json.dumps(
                {
                    "ok": True,
                    "service": "triwhirl-sim-viewer",
                    "sitl": str(self.sitl_executable),
                }
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/live":
            self.stream_live(parsed.query)
            return
        super().do_GET()

    def write_sse(self, event: str, payload: object) -> bool:
        try:
            self.wfile.write(sse_payload(event, payload))
            self.wfile.flush()
            return True
        except OSError as error:
            if is_client_disconnect(error):
                return False
            raise

    def stream_live(self, query: str) -> None:
        params = parse_qs(query)
        profile = params.get("profile", ["nominal"])[0]
        if profile not in {"nominal", "B", "C"}:
            self.send_error(400, "profile must be nominal, B, or C")
            return
        try:
            speed = float(params.get("speed", ["1"])[0])
            fps = float(params.get("fps", ["60"])[0])
        except ValueError:
            self.send_error(400, "speed and fps must be numeric")
            return
        if not math.isfinite(speed) or not (0.1 <= speed <= 10.0):
            self.send_error(400, "speed must be in [0.1, 10]")
            return
        if not math.isfinite(fps) or not (5.0 <= fps <= 120.0):
            self.send_error(400, "fps must be in [5, 120]")
            return

        try:
            samples, gate_pass, summary = run_fresh_simulation(
                self.sitl_executable, profile
            )
        except Exception as error:  # noqa: BLE001 - report backend failure to browser.
            body = str(error).encode("utf-8")
            self.send_response(500)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        try:
            if not self.write_sse(
                "meta",
                {
                    "schema": 1,
                    "source": "fresh native C++ SITL",
                    "profile": profile,
                    "gate_pass": gate_pass,
                    "duration_s": float(samples[-1]["t_s"]),
                    "display_fps_limit": fps,
                    "speed": speed,
                    "scope": (
                        "Production StandupController + provisional local plant; "
                        "simulation evidence only, no physical authority."
                    ),
                },
            ):
                return

            display_period = 1.0 / fps
            next_display_t = 0.0
            wall_anchor = time.perf_counter()
            for sample in samples:
                sim_t = float(sample["t_s"])
                is_last = sample is samples[-1]
                if not is_last and sim_t + 1.0e-12 < next_display_t:
                    continue
                due = wall_anchor + sim_t / speed
                delay = due - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                if not self.write_sse("sample", sample):
                    return
                while next_display_t <= sim_t + 1.0e-12:
                    next_display_t += display_period

            self.write_sse(
                "end",
                {
                    "gate_pass": gate_pass,
                    "summary": summary,
                    "t_s": samples[-1]["t_s"],
                },
            )
        except OSError as error:
            if not is_client_disconnect(error):
                raise
        except Exception as error:  # noqa: BLE001
            try:
                self.write_sse("stream-error", {"message": str(error)})
            except OSError as nested:
                if not is_client_disconnect(nested):
                    raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--exe", help="explicit native triwhirl-standup-sitl path")
    args = parser.parse_args()

    executable = resolve_sitl_executable(args.exe)
    Handler.sitl_executable = executable
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    viewer = f"http://{args.host}:{args.port}/tools/visualization/triwhirl-sim-viewer/"
    print("TriWhirl Simulation Console")
    print(f"SITL:   {executable}")
    print(f"viewer: {viewer}")
    print("Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, exiting.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
