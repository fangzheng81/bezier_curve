set(LIBRARY_NAME "${PROJECT_TARGET_LIB_NAME}")

file(GLOB SOURCE_FILES
  ${CMAKE_CURRENT_LIST_DIR}/bezier/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/bezier/*.hpp
)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(${LIBRARY_NAME}
  SHARED
    ${SOURCE_FILES}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${LIBRARY_NAME}
  PRIVATE
    cxx_std_11
)

target_compile_options(${LIBRARY_NAME}
  PRIVATE
    $<$<CONFIG:Debug>:-O0 -g -Wall -Werror>
    $<$<CONFIG:Release>:-O3>
)

target_link_libraries(${LIBRARY_NAME}
  PUBLIC
    Eigen3::Eigen
)
