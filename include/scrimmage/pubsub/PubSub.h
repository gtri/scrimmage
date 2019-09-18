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

#include <scrimmage/pubsub/Subscriber.h>

#include <map>
#include <list>
#include <string>
#include <memory>

namespace boost {
template <class T> class optional;
}

namespace scrimmage {

class Publisher;
using PublisherPtr = std::shared_ptr<Publisher>;

class EntityPlugin;
using EntityPluginPtr = std::shared_ptr<EntityPlugin>;

class PubSub {
 public:
    PubSub();

    // Key 1: Network name
    // Key 2: Topic name
    // Value : List of NetworkDevicePtr's (publishers or subscribers)
    using TopicMap = std::map<std::string, std::map<std::string, std::list<NetworkDevicePtr>>>;

    TopicMap &pubs() { return pub_map_; }
    TopicMap &subs() { return sub_map_; }

    void add_network_name(const std::string &str);

    boost::optional<std::list<NetworkDevicePtr>> find_devices(const std::string &network_name,
                                                              const std::string &topic_name,
                                                              TopicMap &devs);

    boost::optional<std::list<NetworkDevicePtr>> find_pubs(const std::string &network_name,
                                                           const std::string &topic_name);

    boost::optional<std::list<NetworkDevicePtr>> find_subs(const std::string &network_name,
                                                           const std::string &topic_name);

    template <class T, class CallbackFunc>
    SubscriberBasePtr subscribe(const std::string &network_name,
                                const std::string &topic,
                                CallbackFunc callback,
                                unsigned int max_queue_size,
                                bool enable_queue_size, EntityPluginPtr plugin) {
        if (sub_map_.count(network_name) == 0) {
            print_str(std::string("WARNING: Subscriber unable to connect to network (")
                + network_name + ") on topic (" + topic + ")");
        }

        SubscriberBasePtr sub =
            std::make_shared<Subscriber<T, CallbackFunc>>(
                topic, max_queue_size, enable_queue_size, plugin, callback);
        sub_map_[network_name][topic].push_back(sub);
        return sub;
    }

    PublisherPtr advertise(const std::string &network_name, const std::string &topic,
                           const unsigned int& max_queue_size,
                           const bool& enable_queue_size, EntityPluginPtr plugin);

 protected:
    TopicMap pub_map_;
    TopicMap sub_map_;
    void print_str(const std::string &s);
};
using PubSubPtr = std::shared_ptr<PubSub>;
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PUBSUB_PUBSUB_H_
