cmake_minimum_required(VERSION 3.8)

project(matplotlibcpp-download NONE)

include(ExternalProject)
ExternalProject_Add(
  matplotlibcpp
  SOURCE_DIR "@MATPLOTLIBCPP_DOWNLOAD_ROOT@/matplotlibcpp-src"
  BINARY_DIR "@MATPLOTLIBCPP_DOWNLOAD_ROOT@/matplotlibcpp-build"
  GIT_REPOSITORY
    https://github.com/xmba15/Another_MatplotlibCpp.git
  GIT_TAG
    master
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  TEST_COMMAND ""
)
