set(LIBRARY_NAME RemotePredator_plugin)

set(SRCS
  RemotePredator.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRCS})

target_link_libraries(${LIBRARY_NAME}
  c2overlay-msgs
  scrimmage-core
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_INC_DIR}>
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
)
