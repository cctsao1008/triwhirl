#!/usr/bin/env python3
"""Fail CI if the Route-B sensor-only commissioning profile can actuate a motor."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "main" / "runtime_simplefoc_sensor_commissioning.cpp"
ENTRY = ROOT / "commissioning" / "simplefoc_sensor_app_main.cpp"
MAIN_CMAKE = ROOT / "main" / "CMakeLists.txt"
SIMPLEFOC_CMAKE = ROOT / "components" / "triwhirl_simplefoc" / "CMakeLists.txt"
PLATFORMIO = ROOT / "platformio.ini"


def fail(message: str) -> None:
    raise SystemExit(f"SimpleFOC sensor commissioning contract: {message}")


def require(text: str, tokens: tuple[str, ...], label: str) -> None:
    missing = [token for token in tokens if token not in text]
    if missing:
        fail(f"{label} missing {missing}")


def code_without_comments(text: str) -> str:
    lines: list[str] = []
    for line in text.splitlines():
        code, _separator, _comment = line.partition("//")
        lines.append(code)
    return "\n".join(lines)


def main() -> None:
    profile = PROFILE.read_text(encoding="utf-8")
    entry = ENTRY.read_text(encoding="utf-8")
    main_cmake = MAIN_CMAKE.read_text(encoding="utf-8")
    simplefoc_cmake = SIMPLEFOC_CMAKE.read_text(encoding="utf-8")
    platformio = PLATFORMIO.read_text(encoding="utf-8")

    require(
        profile,
        (
            "TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING",
            '"triwhirl/simplefoc_sensor_path.hpp"',
            '"triwhirl/board.hpp"',
            "initArduino()",
            "SimpleFocSensorPath sensor(config)",
            "sensor.begin()",
            "sensor.service()",
            "sensor.observation()",
            "board::kAs5600SdaGpio",
            "board::kAs5600SclGpio",
            "config.i2c_bus_index = 0",
            "kSensorBusHz = 400000U",
            "vTaskDelayUntil(&last_wake",
            "xQueueOverwrite(snapshot_queue",
            "xQueuePeek(snapshot_queue",
        ),
        "passive profile",
    )

    executable_profile = code_without_comments(profile)
    forbidden_profile_tokens = (
        "simplefoc_motor_backend",
        "SimpleFocMotorBackend",
        "BLDCMotor",
        "BLDCDriver",
        "driver.init(",
        "motor.init(",
        "initFOC(",
        "loopFOC(",
        ".move(",
        ".enable(",
        "setPwm(",
        "bridge.init(",
        "initRuntimeEncoderBus(",
        "Mpu6050",
        "triwhirl::ble",
        "FuzzyAttitudeCommandController",
    )
    leaked = [token for token in forbidden_profile_tokens if token in executable_profile]
    if leaked:
        fail(f"sensor-only profile leaked forbidden runtime/motor operations: {leaked}")

    require(
        entry,
        (
            "TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING",
            'extern "C" void app_main(void)',
            "runSimpleFocSensorCommissioning()",
        ),
        "dedicated commissioning entry",
    )
    entry_code = code_without_comments(entry)
    forbidden_entry = (
        "runtime_main",
        "SimpleFocMotorBackend",
        "bridge",
        "initFOC",
        "loopFOC",
    )
    leaked = [token for token in forbidden_entry if token in entry_code]
    if leaked:
        fail(f"commissioning app entry leaked forbidden path: {leaked}")

    # The commissioning CMake branch must be a minimal graph and must not link
    # the native realtime/motor runtime at all.
    marker = "if(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING)"
    if marker not in main_cmake or "else()" not in main_cmake:
        fail("main CMake has no explicit commissioning branch")
    commissioning_block = main_cmake.split(marker, 1)[1].split("else()", 1)[0]
    require(
        commissioning_block,
        (
            '"../commissioning/simplefoc_sensor_app_main.cpp"',
            '"runtime_simplefoc_sensor_commissioning.cpp"',
            "triwhirl_hw",
            "triwhirl_simplefoc",
            "esp_timer",
            "freertos",
        ),
        "minimal commissioning component graph",
    )
    forbidden_cmake_sources = (
        "runtime_main.cpp",
        "runtime_state.cpp",
        "runtime_motor_task.cpp",
        "runtime_balance.cpp",
        "runtime_standup.cpp",
        "runtime_imu_acquisition.cpp",
        "runtime_ble",
        "runtime_route_b_simplefoc_link_probe.cpp",
    )
    leaked = [token for token in forbidden_cmake_sources if token in commissioning_block]
    if leaked:
        fail(f"commissioning component graph includes native runtime sources: {leaked}")

    # The SimpleFOC component itself also omits the full motor backend in this
    # CMake profile, so the flash image contains only the passive sensor TU.
    sfoc_block = simplefoc_cmake.split(marker, 1)[1].split("else()", 1)[0]
    require(sfoc_block, ('"simplefoc_sensor_path.cpp"',), "passive SimpleFOC graph")
    if "simplefoc_motor_backend.cpp" in sfoc_block:
        fail("sensor-only SimpleFOC graph still compiles the motor backend")

    require(
        platformio,
        (
            "[env:simplefoc-sensor-commissioning]",
            "-DTRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING=ON",
            "-DTRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING=1",
            "-DTRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND=1",
            "Arduino-FOC.git#4f072b365f6e0185adca544071e595834405babc",
            "arduino-esp32.git#76f683d935b390f2805301ec10a5562bbbb37811",
        ),
        "PlatformIO sensor-only environment",
    )
    sensor_env = platformio.split("[env:simplefoc-sensor-commissioning]", 1)[1]
    if "TRIWHIRL_ROUTE_B_SIMPLEFOC_LINK_PROBE" in sensor_env:
        fail("sensor-only environment enabled the inactive full motor link probe")

    print("SimpleFOC sensor commissioning contract: PASS")


if __name__ == "__main__":
    main()
