set(test_files
  test_algorithms.cpp
  test_angles.cpp
  test_collisions.cpp
  test_csv.cpp
  test_delayed_task.cpp
  test_entity_configs.cpp
  test_exponential_filter.cpp
  test_find_mission.cpp
  test_id.cpp
  test_mission_parse.cpp
  test_openai.cpp
  test_params.cpp
  test_parse_utils.cpp
  test_quaternion.cpp
  test_rtree.cpp
  test_shape.cpp
  test_simple.cpp
  test_state.cpp
  test_utilities.cpp
  test_xml_parsers.cpp
  )

if (NOT ENABLE_PYTHON_BINDINGS)
  LIST(REMOVE_ITEM test_files "test_openai.cpp")
else()
  execute_process(
    COMMAND python -c "import gym"
    RESULT_VARIABLE GYM_INSTALLED
    )
  if (NOT GYM_INSTALLED EQUAL 0)
    message("openai gym not found. Not building test_openai.cpp")
    LIST(REMOVE_ITEM test_files "test_openai.cpp")
  endif()
endif()

foreach(test_file ${test_files})
  get_filename_component(test_name ${test_file} NAME_WE)
  add_executable(${test_name} ${test_file})
  #AddExternalTarget(${test_name} EXTERNAL_PROJECT_GTEST )
  add_dependencies(${test_name} scrimmage-core)
  target_link_libraries(${test_name}
    gtest_main
    gmock
    scrimmage-core
    Boost::filesystem # Can remove after switch to use std::filesystem
    )
  if (ENABLE_PYTHON_BINDINGS)
    target_link_libraries(${test_name} scrimmage-python)
  endif()

  add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endforeach()
