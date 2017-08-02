FILE(GLOB test_files test_*.cpp test_*.cc)
foreach(test_file ${test_files})
  get_filename_component(test_name ${test_file} NAME_WE)
  add_executable(${test_name} ${test_file})
  add_dependencies(${test_name} scrimmage)
  target_link_libraries(${test_name}
    ${GTEST_BOTH_LIBRARIES}
    ${GMOCK_BOTH_LIBRARIES}
    scrimmage
    ${PYTHON_LIBRARIES}
    )
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
