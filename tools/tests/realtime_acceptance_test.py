#!/usr/bin/env python3

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.triwhirl_tool.commands.realtime import _evaluate_realtime_acceptance


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
        "mag": "1",
        "ml": "0",
        "mh": "0",
        "vel_valid": "1",
        "imu_ok": "1",
        "fault_mask": "0x00000000",
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
    status["fault_mask"] = "0x00000020"
    failures = evaluate(good_profile(), good_timing(), status)
    assert "fault_mask=0x00000020" in failures

    print("PASS realtime acceptance evaluator")


if __name__ == "__main__":
    main()
