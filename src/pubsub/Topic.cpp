#include <scrimmage/pubsub/NetworkDevice.h>
#include <string.h>

namespace {

class Topic {

  std::string name;
  std::vector<scrimmage::NetworkDevicePtr> subs_;

};
}
