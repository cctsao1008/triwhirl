idf_component_register(
    SRCS "as5600.cpp" "mpu6050.cpp" "runtime_settings.cpp" "three_pwm_bridge.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_mcpwm nvs_flash
)
