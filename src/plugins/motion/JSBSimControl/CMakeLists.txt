#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
if (NOT EXTERNAL AND JSBSIM_FOUND)
  SET (LIBRARY_NAME JSBSimControl_plugin)
  SET (LIB_MAJOR 0)
  SET (LIB_MINOR 0)
  SET (LIB_RELEASE 1)

  file(GLOB SRCS *.cpp)

  ADD_LIBRARY(${LIBRARY_NAME} SHARED
    ${SRCS}
    )

  target_include_directories(${LIBRARY_NAME}
      SYSTEM PUBLIC ${JSBSIM_INCLUDE_DIRS}
    )

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
      scrimmage-core
    ${JSBSIM_LIBRARIES}
    JSBSimModel_plugin
    )

  SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

  set_target_properties(${LIBRARY_NAME} PROPERTIES
    SOVERSION ${LIB_MAJOR}
    VERSION ${_soversion}
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
    )

  install(TARGETS ${LIBRARY_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT ${PROJECT_NAME}-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/${PROJECT_NAME}/plugin_libs
    )

  # Push up the PROJECT_PLUGINS variable
  set(PROJECT_PLUGINS ${PROJECT_PLUGINS} ${LIBRARY_NAME} PARENT_SCOPE)
endif()
