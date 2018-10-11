#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
SET (LIBRARY_NAME ScrimmageOpenAIAutonomy_plugin)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

file(GLOB SRCS *.cpp)

ADD_LIBRARY(${LIBRARY_NAME} SHARED 
  ScrimmageOpenAIAutonomy.cpp
  OpenAIObservations.cpp
  OpenAIActions.cpp
  OpenAIUtils.cpp
  )


TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME}
  PUBLIC
  ${PYTHON_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
  PUBLIC
  scrimmage-core
  scrimmage-python
  )

target_compile_options(${LIBRARY_NAME}
  PRIVATE -Wall -Wno-return-type-c-linkage -std=c++14
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
