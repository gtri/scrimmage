FILE(GLOB test_files test_*.cpp test_*.cc)
foreach(test_file ${test_files})
  get_filename_component(test_name ${test_file} NAME_WE)
  add_executable(${test_name} ${test_file})
  AddExternalTarget(${test_name} EXTERNAL_PROJECT_GTEST )
  add_dependencies(${test_name} scrimmage)
  target_link_libraries(${test_name}
    gtest_main
    scrimmage
    ${PROJECT_PLUGINS}
    ${PYTHON_LIBRARIES}
    )
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
