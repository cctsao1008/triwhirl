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
SIMPLEFOC = ROOT / "components" / "triwhirl_simplefoc"
SENSOR_PATH = SIMPLEFOC / "simplefoc_sensor_path.cpp"
MOTOR_BACKEND = SIMPLEFOC / "simplefoc_motor_backend.cpp"


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
    sensor_path = SENSOR_PATH.read_text(encoding="utf-8")
    motor_backend = MOTOR_BACKEND.read_text(encoding="utf-8")

    require(
        probe,
        (
            '"triwhirl/fuzzy_attitude_command.hpp"',
            '"triwhirl/fuzzy_balance_seed.hpp"',
            '"triwhirl/motor_execution.hpp"',
            '"triwhirl/motor_mailbox.hpp"',
            '"triwhirl/simplefoc_motor_backend.hpp"',
            '"triwhirl/simplefoc_sensor_path.hpp"',
            "SimpleFocSensorPath sensor_only",
            "&triwhirl::simplefoc::SimpleFocSensorPath::begin",
            "&triwhirl::simplefoc::SimpleFocSensorPath::service",
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
        "sensor_only.begin(",
        "sensor_only.service(",
    )
    leaked = [token for token in forbidden_activation if token in executable_probe]
    if leaked:
        fail(f"inactive probe contains runtime activation calls: {leaked}")

    symbol = "triwhirl_route_b_simplefoc_link_probe"
    if symbol in runtime_main:
        fail("native app_main/runtime_main references the Route-B link probe")

    # The passive commissioning class must be structurally incapable of motor
    # actuation. It may own only TwoWire + MagneticSensorI2C operations.
    sensor_code = code_without_line_comments(sensor_path)
    require(
        sensor_code,
        (
            "encoder_bus_.begin(",
            "sensor_.init(&encoder_bus_)",
            "sensor_.update()",
            "sensor_.getAngle()",
            "sensor_.getVelocity()",
            "sensor_.currWireError",
        ),
        "SimpleFOC passive sensor path",
    )
    forbidden_sensor_motor_tokens = (
        "driver_",
        "motor_",
        "initFOC(",
        "loopFOC(",
        ".move(",
        ".enable(",
        "setPwm(",
    )
    leaked = [token for token in forbidden_sensor_motor_tokens if token in sensor_code]
    if leaked:
        fail(f"passive sensor path leaked motor/PWM operation: {leaked}")

    # The full backend must reuse the exact SimpleFOC sensor stage before motor
    # initialization rather than creating a parallel AS5600 reader.
    require(
        motor_backend,
        (
            "sensor_path_.begin()",
            "sensor_path_.service()",
            "motor_.linkSensor(sensor_path_.motorSensorHandle())",
            "driver_.init()",
            "motor_.init()",
            "motor_.initFOC()",
        ),
        "full backend staged sensor ownership",
    )
    if motor_backend.find("sensor_path_.begin()") > motor_backend.find("driver_.init()"):
        fail("full backend initializes the motor driver before the sensor stage")
    if motor_backend.find("motor_.linkSensor(sensor_path_.motorSensorHandle())") > motor_backend.find("motor_.init()"):
        fail("full backend initializes BLDCMotor before linking the shared sensor")

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
