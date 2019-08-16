if (NOT TARGET gtest)
include(googletest)
__fetch_googletest(
    ${PROJECT_SOURCE_DIR}/cmake
    ${PROJECT_BINARY_DIR}/googletest
)
endif()

add_executable(
  ${PROJECT_TARGET_LIB_NAME}_unit_tests
  TestBezier.cpp
)

target_link_libraries(
  ${PROJECT_TARGET_LIB_NAME}_unit_tests
  PUBLIC
    ${PROJECT_TARGET_LIB_NAME}
  PRIVATE
    gtest_main
)

add_test(
  NAME
    ${PROJECT_TARGET_LIB_NAME}_unit_tests
  COMMAND
    $<TARGET_FILE:${PROJECT_TARGET_LIB_NAME}_unit_tests>
)
