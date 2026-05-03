add_executable(test_capture_entity_json test_capture_entity_json.cpp)
target_link_libraries(test_capture_entity_json
  scrimmage-msgs
  protobuf::libprotobuf
)
add_test(NAME test_capture_entity_json COMMAND test_capture_entity_json)
