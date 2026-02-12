# Copyright Robotick contributors
# SPDX-License-Identifier: Apache-2.0

# robotick-core-workloads/cpp/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

if(ROBOTICK_PLATFORM_LINUX)
    project(robotick-core-workloads)
endif()

# ============================
# Detect Platform Macros
# ============================

set(PLATFORM_DETECTED FALSE)

if (DEFINED ROBOTICK_PLATFORM_LINUX)
    message(STATUS "Platform: ROBOTICK_PLATFORM_LINUX")
    set(PLATFORM_DETECTED TRUE)
endif()

if (DEFINED ROBOTICK_PLATFORM_ESP32S3)
    message(STATUS "Platform: ROBOTICK_PLATFORM_ESP32S3")
    set(PLATFORM_DETECTED TRUE)
endif()

if (DEFINED ROBOTICK_PLATFORM_DESKTOP)
    message(STATUS "Platform: ROBOTICK_PLATFORM_DESKTOP")
    set(PLATFORM_DETECTED TRUE)
endif()

if (NOT PLATFORM_DETECTED)
    message(FATAL_ERROR "No platform macro defined! Please define at least one of: ROBOTICK_PLATFORM_LINUX, ROBOTICK_PLATFORM_DESKTOP, ROBOTICK_PLATFORM_ESP32S3.")
endif()

# =========================
# Project-wide Settings
# =========================

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# -----------------------------------------------
# Discover all .cpp files
# -----------------------------------------------
file(GLOB_RECURSE ALL_CPP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/src/robotick/*.cpp
)

# -----------------------------------------------
# Partition out Python-related files
# -----------------------------------------------
set(CORE_SOURCES "")
set(PYTHON_SOURCES "")
foreach(file ${ALL_CPP_SOURCES})
    if(file MATCHES ".*/PythonRuntime.cpp$" OR file MATCHES ".*/PythonWorkload.cpp$")
        if(ROBOTICK_ENABLE_PYTHON)
            list(APPEND CORE_SOURCES ${file})
            list(APPEND PYTHON_SOURCES ${file})
        endif()
    else()
        list(APPEND CORE_SOURCES ${file})
    endif()
endforeach()

# Export results
set(CORE_SOURCES ${CORE_SOURCES} PARENT_SCOPE)
set(PYTHON_SOURCES ${PYTHON_SOURCES} PARENT_SCOPE)

list(LENGTH CORE_SOURCES CORE_COUNT)
list(LENGTH PYTHON_SOURCES PYTHON_COUNT)
message(STATUS "CORE_SOURCES: ${CORE_COUNT}  |  PYTHON_SOURCES: ${PYTHON_COUNT}")

# =========================
# Include the engine
# =========================

option(ROBOTICK_ENGINE_SOURCE_DIR "Path to a local robotick-engine source tree" "")

# 1) Try a parent/side-by-side source tree first
if(ROBOTICK_ENGINE_SOURCE_DIR)
    message(STATUS "Using robotick-engine from source: ${ROBOTICK_ENGINE_SOURCE_DIR}")
    add_subdirectory(${ROBOTICK_ENGINE_SOURCE_DIR} ${CMAKE_BINARY_DIR}/robotick-engine)
    set(ROBOTICK_ENGINE_FOUND TRUE)
endif()

# 2) Error if engine not found
if(NOT ROBOTICK_ENGINE_FOUND)
    message(FATAL_ERROR
        "robotick-engine not found. Please set ROBOTICK_ENGINE_SOURCE_DIR "
        "to a valid local checkout of the engine.")
endif()

# =========================
# Core Framework Library
# =========================

add_library(robotick-core-workloads SHARED ${CORE_SOURCES})

target_compile_options(robotick-core-workloads PRIVATE
    -Wall -Wextra -Werror -fno-rtti
)

set_target_properties(robotick-core-workloads PROPERTIES OUTPUT_NAME robotick-core-workloads)
target_compile_definitions(robotick-core-workloads PRIVATE ROBOTICK_EXPORTS)

target_include_directories(robotick-core-workloads PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_options(robotick-core-workloads PRIVATE -fno-exceptions)

target_link_libraries(robotick-core-workloads PRIVATE robotick-engine yaml-cpp)

# =========================
# Optional Python Workloads
# =========================

include(FetchContent)

if(ROBOTICK_ENABLE_PYTHON)
    message(STATUS "ROBOTICK_ENABLE_PYTHON - ON")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.11.1
    )
    FetchContent_MakeAvailable(pybind11)

    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

    target_compile_definitions(robotick-engine PUBLIC ROBOTICK_ENABLE_PYTHON=1)
    target_link_libraries(robotick-engine PUBLIC pybind11::embed ${Python3_LIBRARIES})
    target_include_directories(robotick-engine PUBLIC ${Python3_INCLUDE_DIRS})
    target_include_directories(robotick-core-workloads PRIVATE ${Python3_INCLUDE_DIRS})

    set(EXCEPTION_ENABLED_SOURCES ${PYTHON_SOURCES})
    list(APPEND EXCEPTION_ENABLED_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/systems/MqttFieldSync.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/workloads/telemetry/MqttClientWorkload.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/workloads/simulation/MuJoCoPhysicsWorkload.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/systems/MqttClient.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/systems/WebServer_desktop.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/systems/Camera_desktop.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/systems/Canvas.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/robotick/systems/Renderer_desktop.cpp
    )

    foreach(source_file IN LISTS EXCEPTION_ENABLED_SOURCES)
        set_source_files_properties(${source_file} PROPERTIES COMPILE_FLAGS "-fexceptions -frtti")
    endforeach()
else()
    message(STATUS "ROBOTICK_ENABLE_PYTHON - OFF")
endif()

# =========================
# Mujoco
# =========================

if (ROBOTICK_PLATFORM_LINUX)
    set(MUJOCO_VERSION "3.1.5")
    set(MUJOCO_ARCHIVE_URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz")
    set(MUJOCO_SDK_ROOT "" CACHE PATH "Optional pre-downloaded MuJoCo SDK directory")

    if (MUJOCO_SDK_ROOT)
        set(_MUJOCO_ROOT ${MUJOCO_SDK_ROOT})
    else()
        include(FetchContent)
        FetchContent_Declare(
            mujoco_sdk
            URL ${MUJOCO_ARCHIVE_URL}
        )
        FetchContent_GetProperties(mujoco_sdk)
        if (NOT mujoco_sdk_POPULATED)
            FetchContent_Populate(mujoco_sdk)
        endif()
        set(_MUJOCO_ROOT ${mujoco_sdk_SOURCE_DIR})
    endif()

    if (NOT EXISTS "${_MUJOCO_ROOT}/lib/libmujoco.so")
        message(FATAL_ERROR "MuJoCo SDK missing libmujoco.so under ${_MUJOCO_ROOT}. Set MUJOCO_SDK_ROOT to a valid install.")
    endif()

    add_library(mujoco SHARED IMPORTED)
    set_target_properties(mujoco PROPERTIES
        IMPORTED_LOCATION "${_MUJOCO_ROOT}/lib/libmujoco.so"
        INTERFACE_INCLUDE_DIRECTORIES "${_MUJOCO_ROOT}/include"
    )
endif()

# =========================
# Additional libs
# =========================

target_link_libraries(robotick-core-workloads PUBLIC nlohmann_json::nlohmann_json)

if(ROBOTICK_PLATFORM_LINUX)
    find_library(EGL_LIBRARY EGL PATHS /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu)
    find_library(OPENGL_LIBRARY GL PATHS /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu)
    if(NOT EGL_LIBRARY OR NOT OPENGL_LIBRARY)
        message(FATAL_ERROR "EGL/OpenGL libraries not found; install libegl1 and libgl1 dev packages.")
    endif()
    
    target_link_libraries(robotick-core-workloads PUBLIC ${EGL_LIBRARY} ${OPENGL_LIBRARY})

    find_package(SDL2 REQUIRED)
    target_link_libraries(robotick-core-workloads PRIVATE civetweb)
    target_link_libraries(robotick-core-workloads PRIVATE SDL2::SDL2)
    target_link_libraries(robotick-core-workloads PRIVATE SDL2_gfx)
    target_link_libraries(robotick-core-workloads PRIVATE SDL2_ttf)
    target_link_libraries(robotick-core-workloads PRIVATE mujoco)
    target_include_directories(robotick-core-workloads PRIVATE /usr/include/SDL2)
    target_include_directories(robotick-core-workloads PRIVATE ${SDL2_GFX_INCLUDE_DIRS})
    target_include_directories(robotick-core-workloads PRIVATE ${SDL2_TTF_INCLUDE_DIRS})
    target_include_directories(robotick-core-workloads PRIVATE ${_MUJOCO_ROOT}/include)
endif()

# =========================
# Add mqtt-c (lightweight C MQTT client)
# =========================
add_library(mqtt-c STATIC
    tests/external/mqtt-c/src/mqtt.c
    tests/external/mqtt-c/src/mqtt_pal.c
)

target_include_directories(mqtt-c PUBLIC tests/external/mqtt-c/include)

target_compile_definitions(mqtt-c PUBLIC
    MQTT_MALLOC=malloc
    MQTT_FREE=free
)

target_link_libraries(robotick-core-workloads PUBLIC mqtt-c)

# ==================================================
# Configure SpeechToText / Whisper.cpp
# ==================================================

# Include Robotick’s Whisper integration module
include(${CMAKE_CURRENT_SOURCE_DIR}/src/robotick/workloads/auditory/SpeechToTextWorkload.cmake)

# Link the generated library into your target
target_link_libraries(robotick-core-workloads PRIVATE whisper)

# =========================
# ESP32 libs (if needed)
# =========================

if(ROBOTICK_PLATFORM_ESP32S3)
    target_include_directories(robotick-core-workloads PRIVATE
        ${IDF_PATH}/components/freertos/FreeRTOS-Kernel/include
        ${IDF_PATH}/components/freertos/FreeRTOS-Kernel/portable/xtensa/include
        ${IDF_PATH}/components/freertos/FreeRTOS-Kernel/portable/xtensa/include/freertos
        ${IDF_PATH}/components/newlib/platform_include
        ${IDF_PATH}/components/esp_common/include
        ${IDF_PATH}/components/esp_system/include
        ${PROJECT_SOURCE_DIR}/esp-idf-dummy
    )
endif()

# =========================
# Unit Tests via Catch2
# =========================

if (ROBOTICK_BUILD_CORE_WORKLOAD_TESTS)
    message(STATUS "ROBOTICK_BUILD_CORE_WORKLOAD_TESTS - ON")
    enable_testing()
    add_subdirectory(tests)

    add_custom_command(TARGET robotick-core-workloads POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:robotick-core-workloads>
            ${CMAKE_BINARY_DIR}/tests/$<CONFIG>
    )
else()
    message(STATUS "ROBOTICK_BUILD_CORE_WORKLOAD_TESTS - OFF")
endif()

# =========================
# Prebuilt OpenCV (Desktop Only)
# =========================

if(ROBOTICK_PLATFORM_DESKTOP)
    find_package(OpenGL REQUIRED)
    find_package(OpenCV REQUIRED)

    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV not found — install with: sudo apt install libopencv-dev")
    else()
        message(STATUS "OpenCV found — version: ${OpenCV_VERSION}")
    endif()

    target_include_directories(robotick-core-workloads PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(robotick-core-workloads PRIVATE ${OpenCV_LIBS})
    target_link_libraries(robotick-core-workloads PRIVATE OpenGL::GL)
else()
    message(STATUS "OpenCV is not enabled on this platform")
endif()

# =========================
# Auto-add License Headers
# =========================

if(ROBOTICK_ENABLE_PYTHON)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    add_custom_target(add_license
        COMMAND ${CMAKE_COMMAND} -E echo "Adding Apache 2.0 headers..."
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/add_license_header.py ${CMAKE_SOURCE_DIR}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()

if (ROBOTICK_BUILD_CORE_WORKLOAD_TESTS)
    include(CTest)
    add_test(NAME robotick_core_workload_tests
             COMMAND robotick_core_workload_tests
             WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/cpp/tests)
endif()
