#include <string.h>

#include "scrimmage/pubsub/NetworkDevice.h"

namespace {

class Topic {

    std::string name;
    std::vector<scrimmage::NetworkDevicePtr> subs_;
};

}  // namespace
