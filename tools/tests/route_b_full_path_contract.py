#!/usr/bin/env python3
"""Source-level guard for the inactive full-fuzzy -> Route-B motor link proof."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROBE = ROOT / "main" / "runtime_route_b_simplefoc_link_probe.cpp"
RUNTIME_MAIN = ROOT / "main" / "runtime_main.cpp"
FUZZY_COMMAND = (
    ROOT
    / "components"
    / "triwhirl_core"
    / "include"
    / "triwhirl"
    / "fuzzy_attitude_command.hpp"
)


def fail(message: str) -> None:
    raise SystemExit(f"Route-B full-path contract: {message}")


def require(text: str, tokens: tuple[str, ...], label: str) -> None:
    missing = [token for token in tokens if token not in text]
    if missing:
        fail(f"{label} missing {missing}")


def code_without_line_comments(text: str) -> str:
    """Remove // comments before checking for executable activation tokens."""
    lines: list[str] = []
    for line in text.splitlines():
        code, _separator, _comment = line.partition("//")
        lines.append(code)
    return "\n".join(lines)


def main() -> None:
    probe = PROBE.read_text(encoding="utf-8")
    runtime_main = RUNTIME_MAIN.read_text(encoding="utf-8")
    fuzzy_command = FUZZY_COMMAND.read_text(encoding="utf-8")

    require(
        probe,
        (
            '"triwhirl/fuzzy_attitude_command.hpp"',
            '"triwhirl/fuzzy_balance_seed.hpp"',
            '"triwhirl/motor_execution.hpp"',
            '"triwhirl/motor_mailbox.hpp"',
            '"triwhirl/simplefoc_motor_backend.hpp"',
            "FuzzyAttitudeCommandController fuzzy_controller",
            "MotorCommandMailbox command_mailbox",
            "MotorObservationMailbox observation_mailbox",
            "MotorExecutionDomain executor",
            "SimpleFocMotorBackend backend",
            "MotorExecutionTask motor_task",
            "observation_mailbox.publish(",
            "observation_mailbox.tryRead(&mechanical_observation)",
            "fuzzy_controller.evaluate(fuzzy_input)",
            "fuzzy_controller.publish(fuzzy_input, &command_mailbox)",
            "command_mailbox.tryRead(&mechanical_command)",
        ),
        "complete inactive target path",
    )

    # The link probe may construct and validate objects, but it must never start
    # hardware service or a FreeRTOS task. Search executable text only so an
    # explanatory comment such as "no executor.service()" cannot false-trigger
    # this safety guard.
    executable_probe = code_without_line_comments(probe)
    forbidden_activation = (
        "executor.begin(",
        "executor.service(",
        "motor_task.start(",
        "motor_control.begin(",
        "motor_control.serviceBackend(",
        "backend.begin(",
    )
    leaked = [token for token in forbidden_activation if token in executable_probe]
    if leaked:
        fail(f"inactive probe contains runtime activation calls: {leaked}")

    symbol = "triwhirl_route_b_simplefoc_link_probe"
    if symbol in runtime_main:
        fail("native app_main/runtime_main references the Route-B link probe")

    # System-control API must remain mechanical. The adapter is allowed to
    # depend on fuzzy + mailbox/upright geometry only; it must not acquire a
    # SimpleFOC or electrical-control dependency while this proof is extended.
    forbidden_fuzzy_includes = (
        "simplefoc",
        "SimpleFOC",
        "voltage_mode_foc",
        "three_pwm_bridge",
    )
    leaked = [token for token in forbidden_fuzzy_includes if token in fuzzy_command]
    if leaked:
        fail(f"fuzzy command boundary leaked motor/electrical dependency: {leaked}")

    require(
        fuzzy_command,
        (
            "FuzzyBalanceController fuzzy_",
            "MotorObservationSnapshot motor{}",
            "publishTarget(output.target_velocity_rad_s",
            "publishStop(input.now_us32)",
        ),
        "mechanical target/stop boundary",
    )

    print("Route-B full-path contract: PASS")


if __name__ == "__main__":
    main()
