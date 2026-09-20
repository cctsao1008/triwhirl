idf_component_register(
    SRCS
        "runtime_parallel_main.cpp"
        "realtime_release.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_driver_gptimer esp_timer freertos
)

# Phase A1 of #32 removes the outer GPTimer source wrapper/macro interception.
# The parallel runtime now calls the release-clock module explicitly; remaining
# legacy source composition is removed in subsequent behavior-preserving steps.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
