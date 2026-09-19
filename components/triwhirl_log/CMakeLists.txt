idf_component_register(
    SRCS "runtime_logger.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_partition freertos
)
