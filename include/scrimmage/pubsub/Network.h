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

#ifndef Network_H_
#define Network_H_

#include <map>
#include <memory>
#include <vector>
#include <unordered_set>
#include <scrimmage/fwd_decl.h>
#include <scrimmage/plugin_manager/Plugin.h>

namespace scrimmage {

class Network : public Plugin {
 public:

    Network();

    virtual void init(std::map<std::string, std::string> &params);
    virtual void distribute(double t, double dt);
    virtual std::unordered_set<int> ping(const PluginPtr &plugin);
    virtual bool ping(const PluginPtr &plugin, int network_id);
    virtual std::string name();
    virtual std::string type();

    void add_publisher(int network_id, PublisherPtr pub, std::string &topic);
    void add_subscriber(int network_id, SubscriberPtr sub, std::string &topic);
    void rm_publisher(int network_id, PublisherPtr pub, std::string &topic);
    void rm_subscriber(int network_id, SubscriberPtr sub, std::string &topic);

    RTreePtr &rtree();

    using DeviceMap = std::map<int, NetworkDevicePtr>;
    using TopicMap = std::map<std::string, DeviceMap>;

    std::unordered_set<int> &network_ids();
    TopicMap &sub_map();

 protected:

    void rm_device(int network_id, std::string topic, NetworkDevicePtr device, TopicMap &map);
    void add_device(int network_id, std::string topic, NetworkDevicePtr device, TopicMap &map);

    TopicMap pub_map_;
    TopicMap sub_map_;
    std::unordered_set<int> network_ids_;
    RTreePtr rtree_;
};

typedef std::shared_ptr<Network> NetworkPtr;
} // namespace scrimmage

#endif // Network_H_
