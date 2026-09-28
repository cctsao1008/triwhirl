cmake_minimum_required(VERSION 3.16)
project(triwhirl_standup_sitl LANGUAGES CXX)

add_executable(triwhirl-standup-sitl
  standup_sitl.cpp
  ../../components/triwhirl_core/standup_controller.cpp
)

add_executable(triwhirl-standup-sitl-disturbance
  standup_sitl_disturbance.cpp
  ../../components/triwhirl_core/standup_controller.cpp
)

add_executable(triwhirl-standup-sitl-live
  standup_sitl_live.cpp
  ../../components/triwhirl_core/standup_controller.cpp
)

foreach(target
    triwhirl-standup-sitl
    triwhirl-standup-sitl-disturbance
    triwhirl-standup-sitl-live)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE
    ../../components/triwhirl_core/include
  )
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()
