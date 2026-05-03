set(PROTO_SRCS TopicTap.proto Commands.proto)
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR})

set(_protoc $<TARGET_FILE:protobuf::protoc>)
set(_grpc_cpp_plugin $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

set(GENERATED_SRCS "")
foreach(proto IN LISTS PROTO_SRCS)
  get_filename_component(name ${proto} NAME_WE)
  set(pb_cc ${GENERATED_DIR}/${name}.pb.cc)
  set(pb_h  ${GENERATED_DIR}/${name}.pb.h)
  set(grpc_cc ${GENERATED_DIR}/${name}.grpc.pb.cc)
  set(grpc_h  ${GENERATED_DIR}/${name}.grpc.pb.h)
  add_custom_command(
    OUTPUT ${pb_cc} ${pb_h} ${grpc_cc} ${grpc_h}
    COMMAND ${_protoc}
    ARGS --grpc_out=${GENERATED_DIR}
         --cpp_out=${GENERATED_DIR}
         -I${CMAKE_CURRENT_SOURCE_DIR}
         --plugin=protoc-gen-grpc=${_grpc_cpp_plugin}
         ${CMAKE_CURRENT_SOURCE_DIR}/${proto}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${proto})
  list(APPEND GENERATED_SRCS ${pb_cc} ${grpc_cc})
endforeach()

add_library(c2overlay-msgs STATIC ${GENERATED_SRCS})
target_include_directories(c2overlay-msgs PUBLIC ${GENERATED_DIR})
target_link_libraries(c2overlay-msgs PUBLIC
  protobuf::libprotobuf
  gRPC::grpc++
)
