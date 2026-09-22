idf_component_register(
    SRCS "attitude_estimator.cpp" "balance_controller.cpp" "safety.cpp" "standup_controller.cpp" "swing_id.cpp" "three_phase_field.cpp" "voltage_mode_foc.cpp" "wheel_kinematics.cpp"
    INCLUDE_DIRS "include"
)

# Keep the control-law hot path optimized even when an existing local sdkconfig
# still carries ESP-IDF's debug-oriented compiler profile.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
