#!/usr/bin/env python3

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.triwhirl_tool.commands.realtime import (
    _classify_drdy_probe,
    _evaluate_realtime_acceptance,
)


def good_profile() -> dict[str, str]:
    return {
        "requests": "5000",
        "completions": "4995",
        "dispatch_failures": "0",
        "read_failures": "0",
        "stale_results": "0",
        "join_timeouts": "0",
        "max_consecutive_misses": "1",
        "period_ge1500": "0",
    }


def good_timing() -> dict[str, str]:
    return {
        "target_us": "1000",
        "iterations": "3000",
        "max_exec_us": "420",
        "max_period_us": "1110",
        "overruns": "0",
        "late_periods": "0",
        "uart_tx_drop_bytes": "0",
        "ble_rx_drop_bytes": "0",
        "ble_tx_drop_bytes": "0",
    }


def good_status() -> dict[str, str]:
    return {
        "status_ok": "1",
        "sample_ok": "1",
        # The known-good vendor hardware can report weak-field diagnostics while
        # RAW_ANGLE and velocity remain usable. These fields are observability,
        # not realtime acceptance gates.
        "mag": "0",
        "ml": "1",
        "mh": "0",
        "agc": "128",
        "magnitude": "300",
        "vel_valid": "1",
        "imu_ok": "1",
        "attitude_ok": "1",
        "fault_mask": "0x00000000",
    }


def good_probe() -> dict[str, str]:
    return {
        "drdy_gpio": "21",
        "drdy_probe_only": "1",
        "drdy_edges": "5010",
        "drdy_consumed": "0",
        "drdy_fallback_reads": "4995",
    }


def evaluate(profile: dict[str, str], timing: dict[str, str], status: dict[str, str]):
    return _evaluate_realtime_acceptance(
        profile,
        timing,
        status,
        min_iterations=1500,
        max_consecutive_misses=1,
        max_exec_us=1000,
        max_period_us=1250,
        min_completion_ratio=0.98,
    )


def main() -> None:
    assert evaluate(good_profile(), good_timing(), good_status()) == []

    profile = good_profile()
    profile["join_timeouts"] = "2"
    failures = evaluate(profile, good_timing(), good_status())
    assert "join_timeouts=2" in failures

    timing = good_timing()
    timing["max_exec_us"] = "1200"
    failures = evaluate(good_profile(), timing, good_status())
    assert "max_exec_us=1200 > 1000" in failures

    status = good_status()
    status["attitude_ok"] = "0"
    failures = evaluate(good_profile(), good_timing(), status)
    assert "attitude_ok=0, expected 1" in failures

    status = good_status()
    status["fault_mask"] = "0x00000020"
    failures = evaluate(good_profile(), good_timing(), status)
    assert "fault_mask=0x00000020" in failures

    classification, rate_hz, ratio = _classify_drdy_probe(
        good_profile(), good_probe(), 5.0
    )
    assert classification == "MATCH"
    assert 999.0 < rate_hz < 1003.0
    assert 1.0 < ratio < 1.01

    no_edges = good_probe()
    no_edges["drdy_edges"] = "0"
    classification, _, ratio = _classify_drdy_probe(
        good_profile(), no_edges, 5.0
    )
    assert classification == "NO_EDGES"
    assert ratio == 0.0

    wrong_rate = good_probe()
    wrong_rate["drdy_edges"] = "2500"
    classification, _, ratio = _classify_drdy_probe(
        good_profile(), wrong_rate, 5.0
    )
    assert classification == "INCONCLUSIVE"
    assert 0.49 < ratio < 0.51

    disabled = good_probe()
    disabled["drdy_gpio"] = "-1"
    classification, _, _ = _classify_drdy_probe(
        good_profile(), disabled, 5.0
    )
    assert classification == "DISABLED"

    print("PASS realtime acceptance evaluator")


if __name__ == "__main__":
    main()
