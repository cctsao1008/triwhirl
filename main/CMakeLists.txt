idf_component_register(
    SRCS "runtime_parallel_main.cpp" "runtime_release.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_driver_gptimer esp_timer freertos
)

# The realtime GPTimer release scheduler is a normal compiled source with an
# explicit interface. The active runtime is built directly from
# runtime_parallel_main.cpp; no outer source-inclusion wrapper remains.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
