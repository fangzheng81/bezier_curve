cmake_minimum_required(VERSION 3.0)

project(libbezier_curve )
set (CMAKE_CXX_STANDARD 11)

set(PROJECT_TARGET_LIB_NAME "bezier_curve")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(build_options)

add_subdirectory(src)
