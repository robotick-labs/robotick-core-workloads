# robotick-engine/examples/cpp/esp32-testbed/main/CMakeLists.txt

if(NOT DEFINED ROBOTICK_PLATFORM_ESP32S3)
    option(ROBOTICK_PLATFORM_ESP32S3 "Build Robotick workloads for ESP32-S3" ON)
endif()
if(NOT DEFINED ROBOTICK_PLATFORM_ESP32S3_M5)
    option(ROBOTICK_PLATFORM_ESP32S3_M5 "Enable M5 board support" OFF)
endif()

set(main_priv_requires robotick-engine robotick-core-workloads)

if(DEFINED ENV{ROBOTICK_PLATFORM_ESP32S3_M5} AND NOT "$ENV{ROBOTICK_PLATFORM_ESP32S3_M5}" STREQUAL "")
    list(APPEND main_priv_requires M5Unified)
endif()

if(DEFINED ROBOTICK_PLATFORM_ESP32S3_M5 AND ROBOTICK_PLATFORM_ESP32S3_M5)
    list(APPEND main_priv_requires M5Unified)
endif()

idf_component_register(
    SRCS "esp32_testbed_main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES ${main_priv_requires}
)
