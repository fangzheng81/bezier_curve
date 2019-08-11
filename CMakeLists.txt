cmake_minimum_required(VERSION 3.8)

project(libbezier_curve)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(utility)
include(googletest)
fetch_googletest(
    ${PROJECT_SOURCE_DIR}/cmake
    ${PROJECT_BINARY_DIR}/googletest
    )

set(PROJECT_TARGET_LIB_NAME "bezier_curve")

add_subdirectory(src)
include(build_options)
