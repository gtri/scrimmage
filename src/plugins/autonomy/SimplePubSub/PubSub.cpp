// Simple Publisher/ Subscriber to test Network
#include <scrimmage/common/Utilities.h>
#include <scrimmage/plugins/autonomy/SimplePubSub/SimpledPubSub.h>
#include <scrimmage/pubsub/PubSub.h>

namespace scrimmage {
namespace autonomy {

void SimplePubSub::init(std::map<std::string, std::string>& params) {
    pub_sphere_ = advertise("SphereNetwork", "PubSubHeartbeat");
    subscribe<std::size_t>(
        "SphereNetwork", "PubSubHeartbeat", [&](auto& msg) { recieved_heartbeat(msg); });
}

bool SimplePubSub::step_autonomy(double t, double dt) {}
}  // namespace autonomy
}  // namespace scrimmage
