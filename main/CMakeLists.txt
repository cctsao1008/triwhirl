idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw esp_driver_uart esp_timer freertos
)
