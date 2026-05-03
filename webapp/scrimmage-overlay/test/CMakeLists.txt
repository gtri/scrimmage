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

add_executable(test_target_assignment_dispatch test_target_assignment_dispatch.cpp)
# Same link list as test_capture_entity_json — c2overlay-msgs brings the typed
# protobuf, plus core/grpc to satisfy transitive deps.
target_link_libraries(test_target_assignment_dispatch
  c2overlay-msgs
  scrimmage-core
  protobuf::libprotobuf
  gRPC::grpc++
)
add_test(NAME test_target_assignment_dispatch COMMAND test_target_assignment_dispatch)
