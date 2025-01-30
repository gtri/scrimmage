#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
set(LIBRARY_NAME ScrimmageOpenAISensor_plugin)
set(LIB_MAJOR 0)
set(LIB_MINOR 0)
set(LIB_RELEASE 1)

SET(SRCS ScrimmageOpenAISensor.cpp )


add_library(${LIBRARY_NAME} SHARED
  ${SRCS}
)

set(_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(${LIBRARY_NAME} PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
)

TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME}
  PUBLIC
  ${PYTHON_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
  PUBLIC
  scrimmage-core
  scrimmage-python
  )

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/${PROJECT_NAME}/plugin_libs
)

# Push up the PROJECT_PLUGINS variable
set(PROJECT_PLUGINS ${PROJECT_PLUGINS} ${LIBRARY_NAME} PARENT_SCOPE)
