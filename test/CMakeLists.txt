set(test_list "")
include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()
set(test_files
    test_algorithms.cpp
    test_angles.cpp
    test_collisions.cpp
    test_delayed_task.cpp
    test_exponential_filter.cpp
    test_find_mission.cpp
    test_id.cpp
    test_params.cpp
    test_quaternion.cpp
    test_rtree.cpp
    test_simple.cpp
    test_state.cpp
    test_utilities.cpp
    test_entity_configs.cpp
    test_openai.cpp
    test_shape.cpp
    )

if (NOT ENABLE_PYTHON_BINDINGS)
    LIST(REMOVE_ITEM test_files "test_openai.cpp")
else()
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "import gym"
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

  AddExternalTarget(${test_name} EXTERNAL_PROJECT_GTEST )
  add_dependencies(${test_name} scrimmage-core)
  target_link_libraries(${test_name}
    gtest_main
    scrimmage-core
    )
  if (ENABLE_PYTHON_BINDINGS)
    target_link_libraries(${test_name} scrimmage-python)
  endif()

  add_test(NAME ${test_name} COMMAND ${test_name})
  list(APPEND test_list "${test_name}")
endforeach()

# Add test executable for all tests for coverage
add_executable(test_all
    ${test_files}
)
add_dependencies(test_all ${test_list} scrimmage-core)
AddExternalTarget(test_all EXTERNAL_PROJECT_GTEST)
target_link_libraries(test_all
  gtest_main
  scrimmage-core
)
if (ENABLE_PYTHON_BINDINGS)
  target_link_libraries(test_all scrimmage-python)
endif()

SETUP_TARGET_FOR_COVERAGE_LCOV(
  NAME test_coverage                        # New target name
  EXECUTABLE test_all -j ${PROCESSOR_COUNT} # Executable in PROJECT_BINARY_DIR
  DEPENDENCIES test_all scrimmage-core      # Dependencies to build first
)
