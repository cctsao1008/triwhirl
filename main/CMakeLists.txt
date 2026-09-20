idf_component_register(
    SRCS "runtime_parallel_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_timer freertos
)

# runtime_parallel_main.cpp wraps the established runtime and substitutes only
# the hard realtime control task so the independent AS5600/MPU6050 I2C buses can
# execute concurrently. Build it for performance regardless of stale local
# sdkconfig optimization choices.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
