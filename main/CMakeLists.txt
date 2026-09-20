idf_component_register(
    SRCS "runtime_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_timer freertos
)

# runtime_main.cpp owns the hard realtime scheduler and directly includes the
# established app runtime implementation. Build it for performance regardless
# of stale local sdkconfig optimization choices.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
