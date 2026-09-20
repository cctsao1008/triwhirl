idf_component_register(
    SRCS "runtime_main.cpp" "runtime_state.cpp" "runtime_command_parser.cpp" "runtime_encoder_acquisition.cpp" "runtime_supervisor_io.cpp" "runtime_snapshot.cpp" "runtime_release.cpp" "runtime_platform.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_driver_gptimer esp_timer freertos
)

# Realtime/startup ownership, shared runtime state, command parsing, sensor
# acquisition, supervisor I/O, snapshot, release-clock, and platform-affinity
# services are compiled as explicit translation units. No .cpp source inclusion
# or symbol-renaming composition shim remains.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
