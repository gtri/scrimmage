add_executable(test_capture_entity_json test_capture_entity_json.cpp)
# Link list mirrors the TopicTap plugin's: scrimmage-msgs.so has transitive
# unresolved deps (scrimmage_proto::*, absl::*, gRPC core) that the plugin
# satisfies via scrimmage-core + gRPC::grpc++. Same combo works here.
target_link_libraries(test_capture_entity_json
  scrimmage-core
  scrimmage-msgs
  protobuf::libprotobuf
  gRPC::grpc++
)
add_test(NAME test_capture_entity_json COMMAND test_capture_entity_json)
