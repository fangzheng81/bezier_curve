set(EX1_NAME "Ex2DPoints")
set(EX2_NAME "Ex3DPoints")

add_executable(
  ${EX1_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/Ex2DPoints.cpp
)

target_link_libraries(
  ${EX1_NAME}
  bezier
)

add_executable(
  ${EX2_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/Ex3DPoints.cpp
)

target_link_libraries(
  ${EX2_NAME}
  bezier
)
