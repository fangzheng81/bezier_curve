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
option(BUILD_VISUALIZATION "Option whether to build examples with visualization" ON)

find_package(Eigen3 QUIET NO_MODULE)

add_subdirectory(src)

if(${BUILD_EXAMPLES})
  message(STATUS "Also build examples")
  add_subdirectory(examples)
endif(${BUILD_EXAMPLES})

option(WITH_DEBUG "Enable debug" OFF)
if(WITH_DEBUG)
  target_compile_definitions(${ProjectTargetLibName}
    PUBLIC
    WITH_DEBUG
  )
endif(WITH_DEBUG)

enable_testing()
add_subdirectory(tests)

option(BUILD_DOC "Build documentation" ON)
if(BUILD_DOC)
  find_package(Doxygen QUIET)
  if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message(STATUS "Doxygen build started")

    add_custom_target( doc_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM )
  else()
    message(STATUS "Doxygen not found")
  endif(DOXYGEN_FOUND)
endif(BUILD_DOC)
