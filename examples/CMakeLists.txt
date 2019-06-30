list(APPEND EXAMPLES
  "Ex2DPoints"
  "Ex3DPoints"
)

list(APPEND USE_LIBRARIES
  bezier
)

function(__build_example example_name use_libraries)
  list(LENGTH use_libraries LIBRARIES_LENGTH)

  add_executable(
    ${example_name}
    ${CMAKE_CURRENT_LIST_DIR}/${example_name}.cpp
  )

  if (LIBRARIES_LENGTH GREATER 0)
    target_link_libraries(
      ${example_name}
      ${use_libraries}
    )
  endif()
endfunction(__build_example example_name use_libraries)

foreach(EXAMPLE ${EXAMPLES})
  __build_example(${EXAMPLE} ${USE_LIBRARIES})
endforeach(EXAMPLE)
