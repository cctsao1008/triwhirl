idf_component_register(
    SRCS "runtime_main.cpp" "runtime_state.cpp" "runtime_balance.cpp" "runtime_standup.cpp" "runtime_trace.cpp" "runtime_command_parser.cpp" "runtime_encoder_acquisition.cpp" "runtime_imu_acquisition.cpp" "runtime_sensor_pipeline.cpp" "runtime_sensor_state.cpp" "runtime_supervisor_io.cpp" "runtime_snapshot.cpp" "runtime_diagnostics.cpp" "runtime_release.cpp" "runtime_platform.cpp"
    INCLUDE_DIRS "."
    REQUIRES triwhirl_core triwhirl_hw triwhirl_ble triwhirl_log esp_driver_uart esp_driver_gptimer esp_driver_gpio esp_timer freertos
)

# Embed the source revision that actually produced the flashed firmware.  The
# trace protocol carries the low 32 bits (eight hex digits) of the commit and a
# dirty-worktree bit so the host does not confuse its own checkout with the
# firmware provenance.
set(TRIWHIRL_GIT_SHA8 "00000000")
set(TRIWHIRL_GIT_DIRTY 1)
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short=8 HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE TRIWHIRL_GIT_SHA_RESULT
        OUTPUT_VARIABLE TRIWHIRL_GIT_SHA_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(TRIWHIRL_GIT_SHA_RESULT EQUAL 0 AND
       TRIWHIRL_GIT_SHA_OUTPUT MATCHES "^[0-9A-Fa-f]{8}$")
        string(TOLOWER "${TRIWHIRL_GIT_SHA_OUTPUT}" TRIWHIRL_GIT_SHA8)
    endif()

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=no
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE TRIWHIRL_GIT_STATUS_RESULT
        OUTPUT_VARIABLE TRIWHIRL_GIT_STATUS_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(TRIWHIRL_GIT_STATUS_RESULT EQUAL 0 AND
       TRIWHIRL_GIT_STATUS_OUTPUT STREQUAL "")
        set(TRIWHIRL_GIT_DIRTY 0)
    endif()

    # Make an incremental idf.py build reconfigure when the checked-out commit
    # moves.  On a normal local branch HEAD itself does not change text, so also
    # watch the symbolic-ref file when it exists.  Detached CI checkouts are
    # covered by .git/HEAD.
    if(EXISTS "${CMAKE_SOURCE_DIR}/.git/HEAD")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                     "${CMAKE_SOURCE_DIR}/.git/HEAD")
    endif()
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" symbolic-ref -q HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE TRIWHIRL_GIT_REF_RESULT
        OUTPUT_VARIABLE TRIWHIRL_GIT_REF
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(TRIWHIRL_GIT_REF_RESULT EQUAL 0 AND
       EXISTS "${CMAKE_SOURCE_DIR}/.git/${TRIWHIRL_GIT_REF}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                     "${CMAKE_SOURCE_DIR}/.git/${TRIWHIRL_GIT_REF}")
    endif()
endif()

# Realtime/startup ownership, shared runtime state/helpers, guarded balance and
# autonomous standup control/trace transport, command parsing, Core-0 sensor
# acquisition/pipeline, supervisor I/O, bounded diagnostics/snapshot,
# release-clock, and platform-affinity services are compiled as explicit
# translation units. No .cpp source inclusion or symbol-renaming composition
# shim remains.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    TRIWHIRL_GIT_SHA32=0x${TRIWHIRL_GIT_SHA8}U
    TRIWHIRL_GIT_DIRTY=${TRIWHIRL_GIT_DIRTY}
)
