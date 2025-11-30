# robotick-core-workloads/platforms/esp32/components/robotick-core-workloads/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

set(ROBOTICK_PLATFORM_ESP32 TRUE)
set(CORE_WORKLOADS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../../cpp)

file(GLOB_RECURSE CORE_SOURCES
    ${CORE_WORKLOADS_ROOT}/src/robotick/*.cpp
)

set(include_dirs
    ${CORE_WORKLOADS_ROOT}/include
)

idf_component_register(
    SRCS ${CORE_SOURCES}
    INCLUDE_DIRS ${include_dirs}
    PRIV_REQUIRES robotick-engine
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC ROBOTICK_PLATFORM_ESP32)
