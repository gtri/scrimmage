#include <cassert>
#include <iostream>
#include <string>

#include <google/protobuf/util/json_util.h>
#include "Commands.pb.h"

int main() {
  // Round-trip: web sends camelCase JSON; protobuf parser must accept it.
  const std::string in = "{\"predatorId\":101,\"targetId\":47}";

  c2overlay_msgs::TargetAssignment msg;
  google::protobuf::util::JsonParseOptions opts;
  opts.ignore_unknown_fields = true;
  auto status = google::protobuf::util::JsonStringToMessage(in, &msg, opts);
  if (!status.ok()) {
    std::cerr << "parse failed: " << status.message() << "\n";
    return 1;
  }
  if (msg.predator_id() != 101) {
    std::cerr << "predator_id mismatch: got " << msg.predator_id() << "\n";
    return 1;
  }
  if (msg.target_id() != 47) {
    std::cerr << "target_id mismatch: got " << msg.target_id() << "\n";
    return 1;
  }

  // Sentinel clear: target_id == 0
  c2overlay_msgs::TargetAssignment clr;
  auto s2 = google::protobuf::util::JsonStringToMessage(
      "{\"predatorId\":101,\"targetId\":0}", &clr, opts);
  if (!s2.ok() || clr.target_id() != 0) {
    std::cerr << "clear sentinel test failed\n";
    return 1;
  }

  std::cout << "OK\n";
  return 0;
}
