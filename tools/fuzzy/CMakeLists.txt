cmake_minimum_required(VERSION 3.15)
project(triwhirl_fuzzy_reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# FuzzyLite is host/SITL reference tooling only. Keep the build minimal and use
# float so the oracle matches the embedded scalar domain as closely as possible.
set(FL_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(FL_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(FL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FL_BUILD_COVERAGE OFF CACHE BOOL "" FORCE)
set(FL_BACKTRACE OFF CACHE BOOL "" FORCE)
set(FL_USE_FLOAT ON CACHE BOOL "" FORCE)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/fuzzylite
    ${CMAKE_BINARY_DIR}/fuzzylite
    EXCLUDE_FROM_ALL
)

add_executable(fuzzy_reference_test reference_test.cpp)
target_include_directories(
    fuzzy_reference_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../components/triwhirl_core/include
)
target_compile_definitions(fuzzy_reference_test PRIVATE FL_USE_FLOAT)
target_link_libraries(fuzzy_reference_test PRIVATE staticTarget)
