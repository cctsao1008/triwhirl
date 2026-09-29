set(SIMPLEFOC_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../third_party/Arduino-FOC")

if(NOT EXISTS "${SIMPLEFOC_ROOT}/src/SimpleFOC.h")
    message(FATAL_ERROR
        "SimpleFOC submodule is missing. Run: git submodule update --init --recursive")
endif()

file(GLOB_RECURSE SIMPLEFOC_SOURCES CONFIGURE_DEPENDS
    "${SIMPLEFOC_ROOT}/src/*.cpp"
)

idf_component_register(
    SRCS ${SIMPLEFOC_SOURCES}
    INCLUDE_DIRS "${SIMPLEFOC_ROOT}/src"
    REQUIRES arduino-esp32
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-parameter)
