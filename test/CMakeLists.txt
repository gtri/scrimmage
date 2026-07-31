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
    test_mission_validation.cpp
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

if(ENABLE_GPU_ACCELERATION AND ENABLE_GPU_TESTS)
    set(test_files ${test_files}
        test_gpu_mapbuffer.cpp
        test_gpu_sphere_network.cpp
        test_compare_gpu_cpu_motion_models.cpp
    )
endif()

foreach(test_file ${test_files})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_executable(${test_name} ${test_file})
    #AddExternalTarget(${test_name} EXTERNAL_PROJECT_GTEST )
    add_dependencies(${test_name} scrimmage-core)
    if (ENABLE_GPU_ACCELERATION AND ENABLE_GPU_TESTS)
        target_link_libraries(${test_name}
            PUBLIC
            Boost::filesystem # Can remove after switch to use std::filesystem
            gmock
            gtest_main
            scrimmage-core
            PRIVATE
            OpenCL::OpenCL
        )
    else()
        target_link_libraries(${test_name}
            PUBLIC
            Boost::filesystem # Can remove after switch to use std::filesystem
            gmock
            gtest_main
            scrimmage-core
        )
    endif()

    if(${test_name} STREQUAL test_gpu_sphere_network)
        target_link_libraries(${test_name}
            PRIVATE
            GPUSphereNetwork_plugin
        )
    endif()

    add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endforeach()
