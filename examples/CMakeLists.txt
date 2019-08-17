include(matplotlibcpp)
__fetch_matplotlibcpp(
  "${PROJECT_SOURCE_DIR}/cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_TARGET_LIB_NAME}_matplotlibcpp"
)

list(APPEND EXAMPLES
  "Ex2DPoints"
  "Ex3DPoints"
  "ExUserDefinedPointType"
  "ExPolynomial1D"
)

include(example_utility)

list(APPEND PUBLIC_LIBS
  ${PROJECT_TARGET_LIB_NAME}
  matplotlib_cpp
)

list(APPEND PRIVATE_LIBS
)

foreach(EXAMPLE ${EXAMPLES})
  __build_example(
    ${EXAMPLE}
    "${PUBLIC_LIBS}"
    "${PRIVATE_LIBS}"
  )
endforeach(EXAMPLE)
