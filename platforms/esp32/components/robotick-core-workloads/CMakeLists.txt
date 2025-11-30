# robotick-core-workloads/platforms/esp32/components/robotick-core-workloads/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ROBOTICK_PLATFORM_ESP32S3)
    option(ROBOTICK_PLATFORM_ESP32S3 "Build Robotick workloads for ESP32-S3" ON)
endif()
if(NOT DEFINED ROBOTICK_PLATFORM_ESP32S3_M5)
    option(ROBOTICK_PLATFORM_ESP32S3_M5 "Enable M5 board support" OFF)
endif()

set(ROBOTICK_PLATFORM_ESP32S3 TRUE)
set(CORE_WORKLOADS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../../cpp)

file(GLOB_RECURSE CORE_SOURCES
    ${CORE_WORKLOADS_ROOT}/src/robotick/*.cpp
)

set(include_dirs
    ${CORE_WORKLOADS_ROOT}/include
)

set(core_priv_requires robotick-engine)

set(is_m5 ${ROBOTICK_PLATFORM_ESP32S3_M5})
if(NOT is_m5 AND DEFINED ENV{ROBOTICK_PLATFORM_ESP32S3_M5} AND NOT "$ENV{ROBOTICK_PLATFORM_ESP32S3_M5}" STREQUAL "")
    set(is_m5 TRUE)
endif()

if(is_m5)
    list(APPEND core_priv_requires M5Unified)
endif()

idf_component_register(
    SRCS ${CORE_SOURCES}
    INCLUDE_DIRS ${include_dirs}
    PRIV_REQUIRES ${core_priv_requires}
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC ROBOTICK_PLATFORM_ESP32S3)

if(is_m5)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC ROBOTICK_PLATFORM_ESP32S3_M5)
endif()
