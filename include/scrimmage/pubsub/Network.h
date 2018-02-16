/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_NETWORK_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_NETWORK_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/pubsub/NetworkDevice.h>

#include <map>
#include <list>
#include <memory>
#include <vector>
#include <string>
#include <unordered_set>

namespace scrimmage {

class Network : public Plugin {
 public:
    Network();

    virtual bool init(std::map<std::string, std::string> &/*mission_params*/,
                      std::map<std::string, std::string> &/*plugin_params*/);
    virtual bool step(std::map<std::string, std::list<NetworkDevicePtr>> &pubs,
                      std::map<std::string, std::list<NetworkDevicePtr>> &subs);
    virtual std::string name();
    virtual std::string type();

    void set_rtree(RTreePtr rtree) { rtree_ = rtree; }
    void set_random(RandomPtr random) { random_ = random; }

 protected:
    RTreePtr rtree_;
    RandomPtr random_;

    // Key 1: Publisher Entity ID
    // Key 2: Subscriber Entity ID
    // Value : Whether the publisher can reach the subscriber with a message
    std::unordered_map<int, std::unordered_map<int, bool>> reachable_map_;

    virtual bool is_reachable(const scrimmage::PluginPtr &pub_plugin,
                              const scrimmage::PluginPtr &sub_plugin);

    virtual bool is_successful_transmission(const scrimmage::PluginPtr &pub_plugin,
                                            const scrimmage::PluginPtr &sub_plugin);
};

typedef std::shared_ptr<Network> NetworkPtr;
using NetworkMap = std::map<std::string, NetworkPtr>;
using NetworkMapPtr = std::shared_ptr<NetworkMap>;

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PUBSUB_NETWORK_H_
