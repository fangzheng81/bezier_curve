function(__fetch_matplotlibcpp download_module_path download_root)
  set(MATPLOTLIBCPP_DOWNLOAD_ROOT ${download_root})
  configure_file(
    ${download_module_path}/matplotlibcpp-download.cmake
    ${download_root}/CMakeLists.txt
    @ONLY
  )
  unset(MATPLOTLIBCPP_DOWNLOAD_ROOT)

  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY
      ${download_root}
  )
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY
      ${download_root}
  )

  add_subdirectory(
    ${download_root}/matplotlibcpp-src
    ${download_root}/matplotlibcpp-build
  )
endfunction()
