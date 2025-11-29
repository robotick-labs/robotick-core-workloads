# Copyright Robotick Labs
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(robotick-core-workloads)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Delegate to the real build logic in cpp/
add_subdirectory(cpp)

if(TARGET code_policy_check)
    add_custom_target(core_workloads_code_policy_check ALL
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/code_policy_check.py ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Checking core-workloads code policy (STL usage + Apache headers)"
    )
    add_dependencies(code_policy_check core_workloads_code_policy_check)
else()
    add_custom_target(code_policy_check ALL
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/code_policy_check.py ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Checking code policy (STL usage + Apache headers)"
    )
endif()
