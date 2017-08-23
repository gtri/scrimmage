#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------

SET (LIBRARY_NAME FixedWing6DOF_plugin)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

file(GLOB SRCS *.cpp)

ADD_LIBRARY(${LIBRARY_NAME} SHARED 
  ${SRCS}
  )

# This is necessary when doing parallel builds (e.g. make -j8)
add_dependencies(${LIBRARY_NAME} scrimmage scrimmage-proto-gen)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SCRIMMAGE_LIBS})

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
add_subdirectory(FixedWing6DOFControllerPID)

if (catkin_FOUND)
  add_subdirectory(FixedWing6DOFControllerROS)
endif()

# Push up the PROJECT_PLUGINS variable
set(PROJECT_PLUGINS ${PROJECT_PLUGINS} ${LIBRARY_NAME} PARENT_SCOPE)



