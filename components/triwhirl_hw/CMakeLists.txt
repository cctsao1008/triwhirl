idf_component_register(
    SRCS "as5600.cpp" "mpu6050.cpp" "three_pwm_bridge.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_mcpwm freertos
)

# Sensor acquisition and PWM updates are on the hard control path.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
