add_executable(
  unit_tests
     TestBezier.cpp
)

target_link_libraries(
  unit_tests
  PUBLIC
     ${PROJECT_TARGET_LIB_NAME}
  PRIVATE
     gtest_main
)

add_test(
  NAME
     unit_tests
  COMMAND
     $<TARGET_FILE:unit_tests>
)
