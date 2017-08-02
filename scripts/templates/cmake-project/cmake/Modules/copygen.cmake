# begin copygen.cmake
function(copygen FROM_DIR TO_DIR)

  # Copy generated header files to include path
  File(GLOB_RECURSE GEN_INCLUDE_FILES RELATIVE
    ${FROM_DIR}
    ${FROM_DIR}/*.h
    )
  foreach(I ${GEN_INCLUDE_FILES})
    configure_file(${FROM_DIR}/${I} ${TO_DIR}/${I} @ONLY)
  endforeach()
endfunction()

copygen(${TO_DIR} ${FROM_DIR})
# end copygen.cmake
