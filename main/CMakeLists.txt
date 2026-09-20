idf_component_register(
    SRCS "runtime_control.cpp" "runtime_release.cpp" "runtime_platform.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_driver_gptimer esp_timer freertos
)

# Realtime control, release-clock, and platform-affinity services are compiled
# as explicit sources. Remaining composition debt is confined to the temporary
# runtime_control.cpp -> runtime_main.cpp -> app_main.cpp include chain.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
