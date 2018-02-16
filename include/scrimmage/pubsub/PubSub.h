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

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_PUBSUB_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_PUBSUB_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <iostream>
#include <map>
#include <list>
#include <string>

using std::cout;
using std::endl;

namespace scrimmage {

class PubSub {
 public:
    PubSub();

    // Key 1: Network name
    // Key 2: Topic name
    // Value : List of NetworkDevicePtr's (publishers or subscribers)
    using TopicMap = std::map<std::string, std::map<std::string, std::list<NetworkDevicePtr>>>;

    TopicMap &pubs() { return pub_map_; }
    TopicMap &subs() { return sub_map_; }

    void add_network_name(std::string str);

    // void add_publisher(int network_id, PublisherPtr pub, std::string &topic);
    // void add_subscriber(SubscriberBasePtr sub, std::string &topic);
    // void rm_publisher(int network_id, PublisherPtr pub, std::string &topic);
    // void rm_subscriber(int network_id, SubscriberBasePtr sub, std::string &topic);

    // using DeviceMap = std::map<int, NetworkDevicePtr>;
    // Key 1: Network name
    // Key 2: Topic name
    // Value : List of NetworkDevicePtr's (publishers or subscribers)

    template <class T>
    SubscriberBasePtr subscribe(std::string &network_name, std::string &topic,
                                int num_msgs,
                                std::function<void(scrimmage::MessagePtr<T>)> callback,
                                PluginPtr plugin) {
        if (sub_map_.count(network_name) == 0) {
            cout << "WARNING: Subscriber unable to connect to network ("
                 << network_name << ") on topic (" << topic << ")" << endl;
        }

        SubscriberBasePtr sub = std::make_shared<Subscriber<T>>(callback);
        sub->set_topic(topic);
        sub->plugin() = plugin;

        sub_map_[network_name][topic].push_back(sub);
        return sub;
    }

    PublisherPtr advertise(std::string &network_name, std::string &topic,
                           int num_msgs, PluginPtr plugin);

 protected:
    // NetworkMapPtr networks_;
    // void rm_device(int network_id, std::string topic, NetworkDevicePtr device, TopicMap &map);
    // void add_device(std::string topic, NetworkDevicePtr device, TopicMap &map);

    TopicMap pub_map_;
    TopicMap sub_map_;
};
using PubSubPtr = std::shared_ptr<PubSub>;
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PUBSUB_PUBSUB_H_
