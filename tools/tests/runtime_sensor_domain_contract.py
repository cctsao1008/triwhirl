#!/usr/bin/env python3
"""Guard the Core-0 sensor-I/O ownership established by #32/C."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "main"
HW = ROOT / "components" / "triwhirl_hw"


def fail(message: str) -> None:
    raise SystemExit(f"runtime sensor-domain contract: {message}")


def require(text: str, tokens: tuple[str, ...], message: str) -> None:
    missing = [token for token in tokens if token not in text]
    if missing:
        fail(f"{message}: {missing}")


def main() -> None:
    runtime = (MAIN / "runtime_main.cpp").read_text(encoding="utf-8")
    encoder_h = (MAIN / "runtime_encoder_acquisition.hpp").read_text(encoding="utf-8")
    encoder_cpp = (MAIN / "runtime_encoder_acquisition.cpp").read_text(encoding="utf-8")
    imu_h = (MAIN / "runtime_imu_acquisition.hpp").read_text(encoding="utf-8")
    imu_cpp = (MAIN / "runtime_imu_acquisition.cpp").read_text(encoding="utf-8")
    sensor_state = (MAIN / "runtime_sensor_state.cpp").read_text(encoding="utf-8")
    mpu_h = (HW / "include" / "triwhirl" / "drivers" / "mpu6050.hpp").read_text(encoding="utf-8")
    mpu_cpp = (HW / "mpu6050.cpp").read_text(encoding="utf-8")

    # Both hardware I2C controllers are allocated from Core 0. I2C0/AS5600 and
    # I2C1/MPU6050 remain separate controllers; only their ownership moved.
    if runtime.count("createI2cMasterBusOnCore(&config, bus, 0)") < 2:
        fail("both sensor I2C buses must allocate their interrupt domain on Core 0")
    if "createI2cMasterBusOnCore(&config, bus, 1)" in runtime:
        fail("sensor I2C interrupt ownership leaked back to realtime Core 1")

    require(runtime,
            ('#include "runtime_encoder_acquisition.hpp"',
             '#include "runtime_imu_acquisition.hpp"',
             '#include "runtime_sensor_state.hpp"',
             "dispatchEncoderAcquisition(", "collectEncoderAcquisition(",
             "dispatchImuAcquisition(", "collectImuAcquisition(",
             "commitRuntimeImuSample("),
            "runtime sensor worker wiring is incomplete")

    if not re.search(r"initEncoderAcquisition\([\s\S]{0,160}?nullptr,\s*0,", runtime):
        fail("AS5600 acquisition worker is not pinned to Core 0")
    if not re.search(r"initImuAcquisition\([\s\S]{0,160}?nullptr,\s*0,", runtime):
        fail("MPU6050 acquisition worker is not pinned to Core 0")

    realtime_begin = runtime.find("void realtimeControlTaskImpl(void*)")
    realtime_end = runtime.find("void triwhirl::runtime::realtimeControlTask(")
    if realtime_begin < 0 or realtime_end <= realtime_begin:
        fail("cannot isolate realtime control task")
    realtime = runtime[realtime_begin:realtime_end]
    if "sampleImu(" in realtime or "imu.readSample(" in realtime:
        fail("blocking MPU6050 sampling returned to realtime Core 1")

    for name, text in (("encoder", encoder_h), ("imu", imu_h)):
        require(text,
                ("requested_at_us", "started_at_us", "completed_at_us"),
                f"{name} acquisition timestamps are incomplete")

    require(encoder_cpp,
            ("request.requested_at_us = static_cast<std::uint32_t>(esp_timer_get_time())",
             "result.started_at_us = static_cast<std::uint32_t>(esp_timer_get_time())",
             "result.completed_at_us = static_cast<std::uint32_t>(esp_timer_get_time())"),
            "AS5600 physical acquisition timestamps regressed")
    require(imu_cpp,
            ("request.requested_at_us = static_cast<std::uint32_t>(esp_timer_get_time())",
             "result.started_at_us = static_cast<std::uint32_t>(esp_timer_get_time())",
             "result.completed_at_us = static_cast<std::uint32_t>(esp_timer_get_time())"),
            "MPU6050 physical acquisition timestamps regressed")

    require(sensor_state,
            ("commitRuntimeImuSample(", "imu_sample = sample;",
             "RuntimeStateEventType::kImuCalibrationComplete"),
            "Core-1 IMU state commit path is incomplete")

    require(mpu_h,
            ("std::atomic<bool> timing_profile_enabled_", "portMUX_TYPE timing_mux_"),
            "MPU timing-profile state is not cross-core safe")
    require(mpu_cpp,
            ("portENTER_CRITICAL(&timing_mux_)", "portEXIT_CRITICAL(&timing_mux_)"),
            "MPU timing-profile implementation lost cross-core synchronization")

    print("runtime sensor-domain contract: PASS")
    print("  sensor_i2c_irq_domain=core0")
    print("  sensor_controllers=i2c0_as5600,i2c1_mpu6050")
    print("  encoder_worker=core0")
    print("  imu_worker=core0")
    print("  blocking_mpu_i2c_in_realtime=no")
    print("  physical_acquisition_timestamps=request,start,complete")
    print("  imu_timing_profile=cross_core_safe")


if __name__ == "__main__":
    main()
