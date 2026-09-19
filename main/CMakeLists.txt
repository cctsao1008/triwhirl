idf_component_register(
    SRCS "runtime_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_timer freertos
)
