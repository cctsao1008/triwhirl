cmake_minimum_required(VERSION 3.16)
project(triwhirl_standup_sitl LANGUAGES CXX)

add_executable(triwhirl-standup-sitl
  standup_sitl.cpp
  ../../components/triwhirl_core/standup_controller.cpp
)

target_compile_features(triwhirl-standup-sitl PRIVATE cxx_std_17)
target_include_directories(triwhirl-standup-sitl PRIVATE
  ../../components/triwhirl_core/include
)

if(MSVC)
  target_compile_options(triwhirl-standup-sitl PRIVATE /W4)
else()
  target_compile_options(triwhirl-standup-sitl PRIVATE -Wall -Wextra -Wpedantic)
endif()
