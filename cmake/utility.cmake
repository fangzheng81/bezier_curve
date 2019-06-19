function(build_example example_name program_name third_party)
  add_executable(
    ${example_name}
    ${program_name}
  )
  target_link_libraries(
    ${example_name}
    ${third_party}
  )
endfunction()
