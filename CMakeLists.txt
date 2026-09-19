cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET "esp32" CACHE STRING "ESP-IDF target")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(triwhirl)
