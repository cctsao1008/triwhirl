#!/usr/bin/env python3
"""Integration smoke test for the continuous TriWhirl WebUI bridge."""

from __future__ import annotations

import json
import sys
import urllib.parse
import urllib.request


def post_json(base: str, path: str, payload: dict[str, object]) -> dict[str, object]:
    request = urllib.request.Request(
        base + path,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=5.0) as response:
        return json.loads(response.read().decode("utf-8"))


def read_event(response) -> tuple[str, dict[str, object]]:
    event_name = ""
    data_lines: list[str] = []
    while True:
        raw = response.readline()
        if not raw:
            raise RuntimeError("SSE stream ended before expected event")
        line = raw.decode("utf-8").rstrip("\r\n")
        if not line:
            if event_name:
                return event_name, json.loads("\n".join(data_lines))
            continue
        if line.startswith("event:"):
            event_name = line.split(":", 1)[1].strip()
        elif line.startswith("data:"):
            data_lines.append(line.split(":", 1)[1].strip())


def main() -> int:
    base = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8765"
    query = urllib.parse.urlencode({"profile": "nominal", "speed": "10", "fps": "120"})
    with urllib.request.urlopen(f"{base}/api/live?{query}", timeout=10.0) as response:
        event, meta = read_event(response)
        assert event == "meta", (event, meta)
        assert meta["mode"] == "LIVE"
        assert meta["duration_s"] is None
        session_id = str(meta["session_id"])
        assert session_id

        sample_count = 0
        while sample_count < 3:
            event, payload = read_event(response)
            if event == "sample":
                sample_count += 1
                assert float(payload["t_s"]) >= 0.0

        accepted = post_json(
            base,
            "/api/disturbance",
            {"session_id": session_id, "kind": "body", "delta_rad_s": 0.2},
        )
        assert accepted["ok"] is True

        disturbance_seen = False
        while not disturbance_seen:
            event, payload = read_event(response)
            if event == "disturbance":
                disturbance_seen = True
                assert payload["kind"] == "body"
                assert abs(float(payload["delta_rad_s"]) - 0.2) < 1.0e-9

        stopped = post_json(base, "/api/stop", {"session_id": session_id})
        assert stopped["ok"] is True

        while True:
            event, payload = read_event(response)
            if event == "end":
                assert payload["reason"] == "user_stop"
                assert float(payload["t_s"]) > 0.0
                break
            if event == "stream-error":
                raise RuntimeError(str(payload))

    print("PASS continuous live WebUI bridge: run -> disturbance -> stop")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
