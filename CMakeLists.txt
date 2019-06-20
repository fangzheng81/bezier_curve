cmake_minimum_required(VERSION 3.0)

project(bezier_curve)
set (CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(utility)
include(googletest)
fetch_googletest(
    ${PROJECT_SOURCE_DIR}/cmake
    ${PROJECT_BINARY_DIR}/googletest
    )

option(BUILD_EXAMPLES "Option whether to build examples" OFF)

find_package(Eigen3 QUIET NO_MODULE)

add_subdirectory(src)

if(${BUILD_EXAMPLES})
  message(STATUS "Also build examples")
  add_subdirectory(examples)
endif(${BUILD_EXAMPLES})

enable_testing()
add_subdirectory(tests)
