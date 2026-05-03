set(LIBRARY_NAME TopicTap_plugin)

set(SRCS
  TopicTap.cpp
  TopicTapServiceImpl.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRCS})

target_link_libraries(${LIBRARY_NAME}
  c2overlay-msgs
  scrimmage-core
  scrimmage-msgs
  protobuf::libprotobuf
  gRPC::grpc++
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_INC_DIR}>
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
)
