#!/usr/bin/env python3
"""Serve the TriWhirl live native-SITL simulation console.

The interactive WebUI has no simulation-duration limit. A native C++ process
owns the fixed-step plant and production StandupController for the whole session;
the browser only displays evidence and sends disturbance/stop commands. The
separate deterministic 10-second batch gate remains in CI as a regression test.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import os
import subprocess
import threading
import uuid
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


def executable_candidates(name: str) -> list[Path]:
    exe = f"{name}.exe" if os.name == "nt" else name
    return [
        ROOT / "build" / "sitl" / "Release" / exe,
        ROOT / "build" / "sitl" / "Debug" / exe,
        ROOT / "build" / "sitl" / exe,
    ]


def resolve_executable(explicit: str | None, name: str) -> Path:
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        if candidate.is_file():
            return candidate
        raise FileNotFoundError(f"SITL executable not found: {candidate}")
    for candidate in executable_candidates(name):
        if candidate.is_file():
            return candidate
    searched = "\n  ".join(str(path) for path in executable_candidates(name))
    raise FileNotFoundError(
        f"{name} executable not found. Build it first. Searched:\n  {searched}"
    )


class LiveSession:
    def __init__(self, process: subprocess.Popen[str]) -> None:
        self.session_id = uuid.uuid4().hex
        self.process = process
        self.write_lock = threading.Lock()

    def send(self, command: str) -> None:
        with self.write_lock:
            if self.process.poll() is not None or self.process.stdin is None:
                raise RuntimeError("native live SITL is no longer running")
            self.process.stdin.write(command + "\n")
            self.process.stdin.flush()

    def terminate(self) -> None:
        if self.process.poll() is not None:
            return
        try:
            self.send("stop")
            self.process.wait(timeout=0.75)
            return
        except Exception:  # noqa: BLE001 - best-effort cleanup.
            pass
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=0.75)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=0.75)


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    sitl_executable: Path
    disturbance_executable: Path
    live_executable: Path
    sessions: dict[str, LiveSession] = {}
    sessions_lock = threading.Lock()

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

    @classmethod
    def register_session(cls, session: LiveSession) -> None:
        with cls.sessions_lock:
            cls.sessions[session.session_id] = session

    @classmethod
    def get_session(cls, session_id: str) -> LiveSession | None:
        with cls.sessions_lock:
            return cls.sessions.get(session_id)

    @classmethod
    def remove_session(cls, session_id: str) -> None:
        with cls.sessions_lock:
            cls.sessions.pop(session_id, None)

    def send_json(self, status: int, payload: object) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json_body(self) -> dict[str, object]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("invalid Content-Length") from error
        if length <= 0 or length > 4096:
            raise ValueError("JSON request body must be 1..4096 bytes")
        raw = self.rfile.read(length)
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("JSON request body must be an object")
        return value

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            self.send_json(
                200,
                {
                    "ok": True,
                    "service": "triwhirl-sim-viewer",
                    "sitl": str(self.sitl_executable),
                    "disturbance_sitl": str(self.disturbance_executable),
                    "live_sitl": str(self.live_executable),
                    "active_sessions": len(self.sessions),
                },
            )
            return
        if parsed.path == "/api/live":
            self.stream_live(parsed.query)
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path not in {"/api/disturbance", "/api/stop"}:
            self.send_error(404)
            return
        try:
            body = self.read_json_body()
            session_id = str(body.get("session_id", ""))
            session = self.get_session(session_id)
            if session is None:
                self.send_json(404, {"ok": False, "error": "live session not found"})
                return

            if parsed.path == "/api/stop":
                session.send("stop")
                self.send_json(202, {"ok": True, "session_id": session_id})
                return

            kind = str(body.get("kind", ""))
            delta = float(body.get("delta_rad_s", float("nan")))
            if kind not in {"body", "wheel"}:
                raise ValueError("kind must be body or wheel")
            if not math.isfinite(delta):
                raise ValueError("delta_rad_s must be finite")
            limit = 5.0 if kind == "body" else 100.0
            if abs(delta) > limit:
                raise ValueError(f"{kind} disturbance exceeds {limit} rad/s")
            session.send(f"{kind} {delta:.9g}")
            self.send_json(
                202,
                {
                    "ok": True,
                    "session_id": session_id,
                    "kind": kind,
                    "delta_rad_s": delta,
                },
            )
        except (ValueError, json.JSONDecodeError) as error:
            self.send_json(400, {"ok": False, "error": str(error)})
        except RuntimeError as error:
            self.send_json(409, {"ok": False, "error": str(error)})

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
            self.send_error(400, "speed/fps must be numeric")
            return
        if not math.isfinite(speed) or not (0.1 <= speed <= 10.0):
            self.send_error(400, "speed must be in [0.1, 10]")
            return
        if not math.isfinite(fps) or not (5.0 <= fps <= 120.0):
            self.send_error(400, "fps must be in [5, 120]")
            return

        command = [
            str(self.live_executable),
            "--profile",
            profile,
            "--speed",
            f"{speed:.9g}",
            "--fps",
            f"{fps:.9g}",
        ]
        try:
            process = subprocess.Popen(
                command,
                cwd=ROOT,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as error:
            self.send_error(500, f"failed to start native live SITL: {error}")
            return

        session = LiveSession(process)
        self.register_session(session)
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
                    "schema": 3,
                    "source": "continuous native C++ SITL",
                    "session_id": session.session_id,
                    "profile": profile,
                    "mode": "LIVE",
                    "duration_s": None,
                    "display_fps_limit": fps,
                    "speed": speed,
                    "scope": (
                        "Production StandupController + provisional local plant; "
                        "native fixed-step simulation continues until explicit Stop."
                    ),
                },
            ):
                return

            assert process.stdout is not None
            for line in process.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    if not self.write_sse(
                        "stream-error",
                        {"message": f"invalid native live-SITL output: {line[:240]}"},
                    ):
                        return
                    continue
                message_type = str(message.get("type", ""))
                if message_type == "sample":
                    if not self.write_sse("sample", message):
                        return
                elif message_type == "disturbance":
                    if not self.write_sse("disturbance", message):
                        return
                elif message_type == "end":
                    self.write_sse("end", message)
                    return
                elif message_type == "error":
                    self.write_sse("stream-error", message)
                else:
                    self.write_sse(
                        "stream-error",
                        {"message": f"unknown native live-SITL message type: {message_type}"},
                    )

            return_code = process.wait(timeout=1.0)
            stderr = process.stderr.read().strip() if process.stderr is not None else ""
            if return_code != 0:
                self.write_sse(
                    "stream-error",
                    {
                        "message": stderr
                        or f"native live SITL exited unexpectedly with {return_code}"
                    },
                )
        except OSError as error:
            if not is_client_disconnect(error):
                raise
        except Exception as error:  # noqa: BLE001 - report backend failure.
            try:
                self.write_sse("stream-error", {"message": str(error)})
            except OSError as nested:
                if not is_client_disconnect(nested):
                    raise
        finally:
            self.remove_session(session.session_id)
            session.terminate()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--exe", help="explicit deterministic triwhirl-standup-sitl path")
    parser.add_argument(
        "--disturbance-exe",
        help="explicit deterministic triwhirl-standup-sitl-disturbance path",
    )
    parser.add_argument(
        "--live-exe", help="explicit continuous triwhirl-standup-sitl-live path"
    )
    args = parser.parse_args()

    Handler.sitl_executable = resolve_executable(args.exe, "triwhirl-standup-sitl")
    Handler.disturbance_executable = resolve_executable(
        args.disturbance_exe, "triwhirl-standup-sitl-disturbance"
    )
    Handler.live_executable = resolve_executable(
        args.live_exe, "triwhirl-standup-sitl-live"
    )
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    viewer = f"http://{args.host}:{args.port}/tools/visualization/triwhirl-sim-viewer/"
    print("TriWhirl Simulation Console")
    print(f"Batch SITL:       {Handler.sitl_executable}")
    print(f"Disturbance SITL: {Handler.disturbance_executable}")
    print(f"Live SITL:        {Handler.live_executable}")
    print(f"viewer:           {viewer}")
    print("WebUI Run continues until Stop. Ctrl+C stops the server.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, stopping live sessions.")
    finally:
        with Handler.sessions_lock:
            sessions = list(Handler.sessions.values())
            Handler.sessions.clear()
        for session in sessions:
            session.terminate()
        server.server_close()


if __name__ == "__main__":
    main()
