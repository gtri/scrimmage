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

#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>
using std::cout;
using std::endl;

namespace scrimmage {

PubSub::PubSub() {
}

void PubSub::add_network_name(std::string network_name) {
    pub_map_[network_name] = std::map<std::string, std::list<NetworkDevicePtr>>();
    sub_map_[network_name] = std::map<std::string, std::list<NetworkDevicePtr>>();
}

PublisherPtr PubSub::advertise(std::string &network_name, std::string &topic,
                               int num_msgs, PluginPtr plugin) {
    if (pub_map_.count(network_name) == 0) {
        cout << "WARNING: Publisher unable to connect to network ("
             << network_name << ") on topic (" << topic << ")" << endl;
    }

    PublisherPtr pub = std::make_shared<Publisher>();
    pub->set_topic(topic);
    pub->plugin() = plugin;
    pub_map_[network_name][topic].push_back(pub);
    return pub;
}

} // namespace scrimmage
