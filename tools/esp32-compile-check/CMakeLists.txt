# robotick-engine/cpp/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

option(ROBOTICK_PLATFORM_ESP32S3 "Build Robotick workloads for ESP32-S3" ON)
option(ROBOTICK_PLATFORM_ESP32S3_M5 "Enable M5 board support" OFF)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../platforms/esp32/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-testbed)
