#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
SET (LIBRARY_NAME RigidBody6DOFControllerROS_plugin)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

if (NOT catkin_FOUND)
  #message(STATUS "ROS not found. Not building RigidBody6DOFControllerROS.")
else() 
    include_directories(${catkin_INCLUDE_DIRS})

    file(GLOB SRCS *.cpp)

    ADD_LIBRARY(${LIBRARY_NAME} SHARED 
      ${SRCS}
      )

    TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
      ${catkin_LIBRARIES}
      )

    SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

    set_target_properties(${LIBRARY_NAME} PROPERTIES 
      SOVERSION ${LIB_MAJOR} 
      VERSION ${_soversion}
      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
      )
    add_dependencies(${LIBRARY_NAME} scrimmage scrimmage-proto-gen)

    install(TARGETS ${LIBRARY_NAME}
      # IMPORTANT: Add the library to the "export-set"
      EXPORT ${PROJECT_NAME}-targets
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/${PROJECT_NAME}/plugin_libs
    )

    # Push up the PROJECT_PLUGINS variable
    set(PROJECT_PLUGINS ${PROJECT_PLUGINS} ${LIBRARY_NAME} PARENT_SCOPE)

endif()
