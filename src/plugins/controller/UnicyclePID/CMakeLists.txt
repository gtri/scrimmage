#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
set(LIBRARY_NAME UnicyclePID_plugin)
set(LIB_MAJOR 0)
set(LIB_MINOR 0)
set(LIB_RELEASE 1)

file(GLOB SRCS *.cpp)

add_library(${LIBRARY_NAME} SHARED
  ${SRCS}
)

target_compile_options(${LIBRARY_NAME}
  PRIVATE -Wall -std=c++14 -Wno-return-type-c-linkage
)

target_link_libraries(${LIBRARY_NAME}
  scrimmage-core
)

set(_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(${LIBRARY_NAME} PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${PROJECT_INC_DIR}>
)

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/${PROJECT_NAME}/plugin_libs
)

# Push up the PROJECT_PLUGINS variable
set(PROJECT_PLUGINS ${PROJECT_PLUGINS} ${LIBRARY_NAME} PARENT_SCOPE)
