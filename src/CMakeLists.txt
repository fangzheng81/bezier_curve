set(LIBRARY_NAME "bezier")
file(GLOB SOURCE_FILES
  ${CMAKE_CURRENT_LIST_DIR}/bezier/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/bezier/*.hpp
)

set(USE_LIBS
  Eigen3::Eigen
)

function(__build_library)
  add_library(${LIBRARY_NAME}
    SHARED
    ${SOURCE_FILES}
    )

  target_link_libraries(${LIBRARY_NAME}
    ${USE_LIBS}
    )

  target_include_directories(${LIBRARY_NAME}
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    )
endfunction(__build_library)

if(TARGET Eigen3::Eigen)
  message(STATUS "Found Eigen")
  __build_library()
else()
  message(STATUS "Eigen not found. Quit building bezier")
endif (TARGET Eigen3::Eigen)
