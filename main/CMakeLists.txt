idf_component_register(
    SRCS "runtime_gptimer_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_driver_gptimer esp_timer freertos
)

# runtime_gptimer_main.cpp wraps the parallel sensor runtime and replaces only
# the 1 kHz release wait with a hardware GPTimer notification. The parallel
# AS5600/MPU6050 acquisition and established supervisor/swing logic remain the
# runtime authority.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
