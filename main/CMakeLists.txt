idf_component_register(
    SRCS "runtime_parallel_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_driver_gptimer esp_timer freertos
)

# The realtime GPTimer release scheduler is now an explicit API used by the
# parallel runtime. This removes the outer runtime_gptimer_main.cpp source-
# inclusion/macro wrapper while preserving the established supervisor/swing
# behavior for the first #32 refactor slice.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
