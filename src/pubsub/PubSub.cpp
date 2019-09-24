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

#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>

#include <boost/optional.hpp>

using std::cout;
using std::endl;

namespace scrimmage {

PubSub::PubSub() {
}

void PubSub::add_network_name(const std::string &network_name) {
    pub_map_[network_name] = std::map<std::string, std::list<NetworkDevicePtr>>();
    sub_map_[network_name] = std::map<std::string, std::list<NetworkDevicePtr>>();
}

PublisherPtr PubSub::advertise(const std::string &network_name,
                               const std::string &topic,
                               const unsigned int& max_queue_size,
                               const bool& enable_queue_size,
                               EntityPluginPtr plugin) {
    if (pub_map_.count(network_name) == 0) {
        cout << "WARNING: " << plugin->name()
             << " - Publisher unable to connect to network ("
             << network_name << ") on topic (" << topic << ")" << endl;
    }

    PublisherPtr pub = std::make_shared<Publisher>(topic, max_queue_size,
                                                   enable_queue_size, plugin);
    pub_map_[network_name][topic].push_back(pub);
    return pub;
}

boost::optional<std::list<NetworkDevicePtr>> PubSub::find_devices(
    const std::string &network_name, const std::string &topic_name, TopicMap &devs) {

    auto it_network = devs.find(network_name);
    if (it_network == devs.end()) {
         cout << "Failed to find network while setting up device." << endl;
         cout << "Network name: " << network_name << endl;
         cout << "Topic name: " << topic_name << endl;
    } else {
        auto it_topic_pub = it_network->second.find(topic_name);
        if (it_topic_pub == it_network->second.end()) {
            cout << "Failed to find topic while setting up device." << endl;
            cout << "Network name: " << network_name << endl;
            cout << "Topic name: " << topic_name << endl;
        } else {
            return boost::optional<std::list<NetworkDevicePtr>>(it_topic_pub->second);
        }
    }
    return boost::none;
}

boost::optional<std::list<NetworkDevicePtr>> PubSub::find_pubs(
    const std::string &network_name, const std::string &topic_name) {
    return find_devices(network_name, topic_name, pub_map_);
}

boost::optional<std::list<NetworkDevicePtr>> PubSub::find_subs(
    const std::string &network_name, const std::string &topic_name) {
    return find_devices(network_name, topic_name, sub_map_);
}

void PubSub::print_str(const std::string &s) {
    std::cout << s << std::endl;
}
} // namespace scrimmage
