#include <cassert>
#include <iostream>
#include <string>

#include <google/protobuf/util/json_util.h>
#include "scrimmage/msgs/Capture.pb.h"

int main() {
  scrimmage_msgs::CaptureEntity msg;
  msg.set_source_id(5);
  msg.set_target_id(47);

  std::string out;
  google::protobuf::util::JsonPrintOptions opts;
  opts.preserve_proto_field_names = false;
  auto status = google::protobuf::util::MessageToJsonString(msg, &out, opts);
  if (!status.ok()) {
    std::cerr << "MessageToJsonString failed: " << status.message() << "\n";
    return 1;
  }

  // Default JSON casing converts source_id -> sourceId, target_id -> targetId.
  const std::string expected = "{\"sourceId\":5,\"targetId\":47}";
  if (out != expected) {
    std::cerr << "Mismatch.\n  expected: " << expected << "\n  got:      " << out << "\n";
    return 1;
  }
  std::cout << "OK: " << out << "\n";
  return 0;
}
