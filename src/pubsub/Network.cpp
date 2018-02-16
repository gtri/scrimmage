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

#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/SubscriberBase.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>

#include <memory>

namespace sc = scrimmage;

namespace scrimmage {
Network::Network() : Plugin(), rtree_(std::make_shared<RTree>()) {
}

bool Network::init(std::map<std::string, std::string> &mission_params,
                   std::map<std::string, std::string> &plugin_params) {
    return true;
}

// void Network::rm_device(int network_id, std::string topic, NetworkDevicePtr device, TopicMap &map) {
//     // check to see if this device exists
//     TopicMap::iterator topic_it = map.find(topic);
//     if (topic_it == map.end()) {
//         return;
//     }
//
//     DeviceMap::iterator id_it = topic_it->second.find(network_id);
//     if (id_it == topic_it->second.end()) {
//         return;
//     }
//
//     // remove the device now that it has been found
//     network_ids_.erase(network_id);
//
//     topic_it->second.erase(id_it);
//     if (topic_it->second.empty()) {
//         map.erase(topic_it);
//     }
// }
//
// void Network::add_device(std::string topic, NetworkDevicePtr device, TopicMap &map) {
//    // Find the topic
//    TopicMap::iterator topic_it = map.find(topic);
//    if (topic_it == map.end()) {
//        map[topic][network_id] = device;
//    } else {
//        DeviceMap::iterator id_it = topic_it->second.find(network_id);
//        if (id_it == topic_it->second.end()) {
//            topic_it->second[network_id] = device;
//        } else {
//            id_it->second = device;
//        }
//    }
//    network_ids_.insert(network_id);
//}
//
// void Network::rm_publisher(int network_id, PublisherPtr pub, std::string &topic) {
//     rm_device(network_id, topic, pub, pub_map_);
// }
//
// void Network::rm_subscriber(int network_id, SubscriberBasePtr sub, std::string &topic) {
//     rm_device(network_id, topic, sub, sub_map_);
// }
//
// void Network::add_publisher(int network_id, PublisherPtr pub, std::string &topic) {
//     add_device(network_id, topic, pub, pub_map_);
// }
//

bool Network::step(std::map<std::string, std::list<NetworkDevicePtr>> &pubs,
                   std::map<std::string, std::list<NetworkDevicePtr>> &subs) {
    reachable_map_.clear();

    // For all publisher topic names
    for (auto &pub_kv : pubs) {
        std::string topic = pub_kv.first;

        // For all publisher devices with topic name
        for (NetworkDevicePtr &pub : pub_kv.second) {
            auto msgs = pub->msgs<sc::MessageBase>(true);
            if (msgs.empty()) {
                continue;
            }

            // For all subscribers on this topic
            for (NetworkDevicePtr &sub : subs[topic]) {
                if (is_reachable(pub->plugin(), sub->plugin())) {
                    for (auto &msg : msgs) {
                        if (is_successful_transmission(pub->plugin(),
                                                       sub->plugin())) {
                            msg->time = time_->t();
                            sub->add_msg(msg);
                        }
                    }
                }
            }
        }
    }
    return true;
}

bool Network::is_reachable(const scrimmage::PluginPtr &pub_plugin,
                           const scrimmage::PluginPtr &sub_plugin) {
    return false;
}

bool Network::is_successful_transmission(const scrimmage::PluginPtr &pub_plugin,
                                         const scrimmage::PluginPtr &sub_plugin) {
    return false;
}

std::string Network::name() {
    return name_;
}

std::string Network::type() {
    return std::string("Network");
}

} // namespace scrimmage
