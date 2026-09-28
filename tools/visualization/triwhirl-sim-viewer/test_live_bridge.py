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
    query = urllib.parse.urlencode(
        {
            "scenario": "full-standup",
            "profile": "nominal",
            "speed": "10",
            "fps": "120",
        }
    )
    with urllib.request.urlopen(f"{base}/api/live?{query}", timeout=10.0) as response:
        event, meta = read_event(response)
        assert event == "meta", (event, meta)
        assert meta["mode"] == "LIVE"
        assert meta["duration_s"] is None
        assert meta["scenario"] == "full-standup"
        session_id = str(meta["session_id"])
        assert session_id

        first_sample: dict[str, object] | None = None
        saw_swing_high = False
        saw_swing_low = False
        saw_positive_rate = False
        saw_negative_rate = False
        min_center_x = float("inf")
        max_center_x = float("-inf")

        while True:
            event, payload = read_event(response)
            if event == "stream-error":
                raise RuntimeError(str(payload))
            if event != "sample":
                continue
            if first_sample is None:
                first_sample = payload
                assert abs(float(payload["true_error_deg"]) + 60.0) < 0.05
                assert abs(float(payload["true_contact_y_m"])) < 1.0e-9
                assert 0.02 < float(payload["true_body_center_y_m"]) < 0.06
                assert abs(float(payload["geometry_width_m"]) - 0.075) < 1.0e-12

            phase = str(payload["phase"])
            saw_swing_high = saw_swing_high or phase == "swing_high"
            saw_swing_low = saw_swing_low or phase == "swing_low"
            rate = float(payload["true_theta_rate_rad_s"])
            saw_positive_rate = saw_positive_rate or rate > 0.05
            saw_negative_rate = saw_negative_rate or rate < -0.05
            center_x = float(payload["true_body_center_x_m"])
            min_center_x = min(min_center_x, center_x)
            max_center_x = max(max_center_x, center_x)
            assert abs(float(payload["true_contact_y_m"])) < 1.0e-8
            assert "true_contact_body_x_m" in payload
            assert "true_contact_body_y_m" in payload

            if float(payload["t_s"]) >= 3.5:
                break

        assert first_sample is not None
        assert saw_swing_high
        assert saw_swing_low
        assert saw_positive_rate and saw_negative_rate
        assert max_center_x - min_center_x > 0.001

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
            elif event == "stream-error":
                raise RuntimeError(str(payload))

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

    print(
        "PASS continuous geometry-derived WebUI bridge: "
        "rest -> rolling swing/reversal -> disturbance -> stop"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
