# set(EX1_NAME "BasicBezier")
set(EX1_NAME "Ex2DPoints")

set(USE_LIBARIES bezier)

if(${BUILD_VISUALIZATION})
  if(${VTK_FOUND})
    set(USE_LIBRARIES ${USE_LIBARIES} ${VTK_LIBARIES})
    include_directories(${VTK_INCLUDE_DIRS})
  else()
    message(STATUS "VTK not found, thus visualization part will not be built.")
endif(${VTK_FOUND})
endif(${BUILD_VISUALIZATION})

add_executable(
  ${EX1_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/Ex2DPoints.cpp
  # ${CMAKE_CURRENT_LIST_DIR}/Ex1.cpp
)

target_link_libraries(
  ${EX1_NAME}
  ${USE_LIBRARIES}
)
