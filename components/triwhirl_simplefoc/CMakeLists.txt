idf_component_register(
    SRCS "simplefoc_motor_backend.cpp"
    INCLUDE_DIRS "include"
    REQUIRES triwhirl_core
)

# This component is intentionally present in both build graphs. In the native
# ESP-IDF build TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND is absent, so the source is
# inert and introduces no Arduino/SimpleFOC dependency. The Route-B PlatformIO
# graph defines the flag explicitly and resolves the pinned Arduino/SimpleFOC
# libraries, proving the production-shaped backend without activating it.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
