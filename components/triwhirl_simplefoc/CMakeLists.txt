idf_component_register(
    SRCS "simplefoc_sensor_path.cpp" "simplefoc_motor_backend.cpp"
    INCLUDE_DIRS "include"
    REQUIRES triwhirl_core
)

# Route-B injects Arduino-ESP32 as the `framework-arduinoespressif32` IDF
# component. Declare it as an optional public dependency so that, when present,
# this backend inherits Arduino's public include/dependency graph (including
# esp_timer) instead of copying ESP-IDF internal include paths. Native idf.py
# builds do not contain that component, so the dependency is simply absent and
# the backend remains inert there.
idf_component_optional_requires(PUBLIC framework-arduinoespressif32)

# This component is intentionally present in both build graphs. In the native
# ESP-IDF build TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND is absent, so the sources are
# inert and introduce no Arduino/SimpleFOC dependency. Route-B explicitly
# enables both the passive sensor path and the full motor backend against the
# pinned Arduino/SimpleFOC libraries.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
