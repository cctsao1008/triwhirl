if(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING)
    # The passive commissioning image intentionally has no TriWhirl motor
    # backend translation unit at all. Only the SimpleFOC-owned AS5600 path is
    # compiled into this component.
    set(TRIWHIRL_SIMPLEFOC_SRCS "simplefoc_sensor_path.cpp")
else()
    set(TRIWHIRL_SIMPLEFOC_SRCS
        "simplefoc_sensor_path.cpp"
        "simplefoc_motor_backend.cpp"
    )
endif()

idf_component_register(
    SRCS ${TRIWHIRL_SIMPLEFOC_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES triwhirl_core
)

# Route-B injects Arduino-ESP32 as the `framework-arduinoespressif32` IDF
# component. Declare it as an optional public dependency so that, when present,
# this component inherits Arduino's public include/dependency graph (including
# esp_timer) instead of copying ESP-IDF internal include paths. Native idf.py
# builds do not contain that component, so the dependency is simply absent and
# the backend remains inert there.
idf_component_optional_requires(PUBLIC framework-arduinoespressif32)

# Native IDF builds omit TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND, so the source body
# stays inert. Normal Route-B builds compile the passive sensor path plus full
# motor backend; the dedicated sensor commissioning profile compiles only the
# passive path above.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
