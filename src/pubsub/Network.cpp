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

#include <memory>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Entity.h>
#include <boost/range/adaptor/map.hpp>

namespace ba = boost::adaptors;
namespace scrimmage {

Network::Network() : rtree_(std::make_shared<RTree>()) {}

void Network::init(std::map<std::string, std::string> &params) {return;}

void Network::rm_device(int network_id, std::string topic, NetworkDevicePtr device, TopicMap &map) {
    // check to see if this device exists
    TopicMap::iterator topic_it = map.find(topic);
    if (topic_it == map.end()) {
        return;
    }

    DeviceMap::iterator id_it = topic_it->second.find(network_id);
    if (id_it == topic_it->second.end()) {
        return;
    }

    // remove the device now that it has been found
    network_ids_.erase(network_id);

    topic_it->second.erase(id_it);
    if (topic_it->second.empty()) {
        map.erase(topic_it);
    }
}

void Network::add_device(int network_id, std::string topic, NetworkDevicePtr device, TopicMap &map) {
    TopicMap::iterator topic_it = map.find(topic);
    if (topic_it == map.end()) {
        map[topic][network_id] = device;
    } else {
        DeviceMap::iterator id_it = topic_it->second.find(network_id);
        if (id_it == topic_it->second.end()) {
            topic_it->second[network_id] = device;
        } else {
            id_it->second = device;
        }
    }
    network_ids_.insert(network_id);
}

void Network::rm_publisher(int network_id, PublisherPtr pub, std::string &topic) {
    rm_device(network_id, topic, pub, pub_map_);
}

void Network::rm_subscriber(int network_id, SubscriberPtr sub, std::string &topic) {
    rm_device(network_id, topic, sub, sub_map_);
}

void Network::add_publisher(int network_id, PublisherPtr pub, std::string &topic) {
    add_device(network_id, topic, pub, pub_map_);
}

void Network::add_subscriber(int network_id, SubscriberPtr sub, std::string &topic) {
    add_device(network_id, topic, sub, sub_map_);
}

void Network::distribute(double t, double dt) {
    for (auto &pub_map_kv : pub_map_) {

        std::string topic = pub_map_kv.first;

        for (NetworkDevicePtr &pub : ba::values(pub_map_kv.second)) {
            if (pub->msg_list().empty()) {
                continue;
            }
            auto beg = pub->msg_list().begin();
            auto end = pub->msg_list().end();

            for (NetworkDevicePtr &sub : ba::values(sub_map_[topic])) {
                sub->msg_list().insert(sub->msg_list().end(), beg, end);
            }

            pub->msg_list().clear();
        }
    }
}

std::string Network::name() {
    return std::string("Network");
}

std::string Network::type() {
    return std::string("Network");
}

RTreePtr &Network::rtree() {
    return rtree_;
}

Network::TopicMap &Network::sub_map() {
    return sub_map_;
}

std::unordered_set<int> Network::ping(const PluginPtr &plugin) {
    return network_ids_;
}

bool Network::ping(const PluginPtr &plugin, int network_id) {
    return network_ids_.count(network_id) != 0;
}

} // namespace scrimmage
