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

#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/msgs/Simple.pb.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/NetworkDevice.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/SubscriberBase.h>

#include <memory>
#include <iostream>

namespace scrimmage {

Network::Network() : EntityPlugin(), rtree_(std::make_shared<RTree>()) {}

bool Network::init(std::map<std::string, std::string> &mission_params,
                   std::map<std::string, std::string> &plugin_params) {
    return network_init(mission_params, plugin_params);
}

bool Network::network_init(std::map<std::string, std::string> &mission_params,
                           std::map<std::string, std::string> &plugin_params) {

    auto setup_counts = [] (std::string str,
                            std::map<std::string, std::string> &plugin_params,
                            std::map<std::string, unsigned int> &counts,
                            bool &monitor_all) {
        std::string topics_str = get<std::string>(str, plugin_params, "");

        auto topics = str2container<std::vector<std::string>>(topics_str, ", ");

        if (std::end(topics) != std::find(std::begin(topics),
                                          std::end(topics), "*")) {
            counts["*"] = 0;
            monitor_all = true;
        }

        for (std::string const &topic : topics) {
            counts[topic] = 0;
        }
    };

    // Get communication delay for the network (default is 0)
    comm_delay_ = scrimmage::get<double>("comm_delay", plugin_params, comm_delay_);
    is_stochastic_delay_ = scrimmage::get<bool>("is_stochastic_delay", plugin_params, is_stochastic_delay_);

    setup_counts("monitor_publisher_topics", plugin_params, pub_counts_, monitor_all_pubs_);
    setup_counts("monitor_subscriber_topics", plugin_params, sub_counts_, monitor_all_subs_);

    // Should we write a CSV file? What values should be written?
    std::string filename = get<std::string>("csv_filename", plugin_params, "");
    if (filename != "") {
        std::cout << "Writing to CSV..." << std::endl;
        write_csv_ = true;
        csv_.open_output(mp_->log_dir() + "/" + filename);

        std::string headers = "t,";
        for (auto &kv : pub_counts_) {
            headers += kv.first + "_Pub_Count,";
        }
        for (auto &kv : sub_counts_) {
            headers += kv.first + "_Sub_Count,";
        }

        csv_.set_column_headers(headers);
    }
    return true;
}

bool Network::step(std::map<std::string, std::list<NetworkDevicePtr>> &pubs,
                   std::map<std::string, std::list<NetworkDevicePtr>> &subs) {
    reachable_map_.clear();

    // Reset msg pub / sub counts
    for (auto &kv : pub_counts_) {
        kv.second = 0;
    }
    for (auto &kv : sub_counts_) {
        kv.second = 0;
    }

    // Get references to the "all topics" string for faster access later
    auto it_all_pub = pub_counts_.find("*");
    auto it_all_sub = sub_counts_.find("*");


    // number of messages delivered (may be >1 if delivered with delay)
    int n_delivered = 0;

    // For all publisher topic names
    for (auto &pub_kv : pubs) {
        std::string topic = pub_kv.first;

        // Get reference to this specific topic in the pub_counts_ map for
        // faster access later
        auto it_pub_topic = pub_counts_.find(topic);
        bool valid_pub_topic = (it_pub_topic != pub_counts_.end());

        // Get reference to this specific topic in the sub_counts_ map for
        // faster access later
        auto it_sub_topic = sub_counts_.find(topic);
        bool valid_sub_topic = (it_sub_topic != sub_counts_.end());

        // For all publisher devices with topic name
        for (NetworkDevicePtr &pub : pub_kv.second) {
            pub->enforce_queue_size();

            auto msgs = pub->pop_msgs<MessageBase>();
            if (msgs.empty()) {
                continue;
            }

            if (monitor_all_pubs_) {
                // Accumulate published message counts on all topics
                it_all_pub->second += msgs.size();
            }

            if (valid_pub_topic) {
                // Accumulate published message counts on specific topic
                it_pub_topic->second += msgs.size();
            }

            // For all subscribers on this topic
            for (NetworkDevicePtr &sub : subs[topic]) {

                if (sub->undelivered_msg_list_size() > 0) {
                    // deliver undelivered messages if delay time has passed
                    n_delivered = sub->deliver_undelivered_msg(time_->t(), is_stochastic_delay_);
                    if (monitor_all_subs_) {
                        // Accumulate received msg counts on all topics
                        it_all_sub->second += n_delivered;
                    }
                    if (valid_sub_topic) {
                        // Accumulate received msg counts on specific topic
                        it_sub_topic->second += n_delivered;
                    }
                }

                if (is_reachable(pub->plugin(), sub->plugin())) {
                    for (auto &msg : msgs) {
                        if (is_successful_transmission(pub->plugin(),
                                                       sub->plugin())) {

                            double msg_delay = get_transmission_delay();

                            if (msg_delay < 0) {

                                msg->time = time_->t();
                                sub->add_msg(msg);

                                if (monitor_all_subs_) {
                                    // Accumulate received message counts on all topics
                                    it_all_sub->second += 1;
                                }

                                if (valid_sub_topic) {
                                    // Accumulate received message counts on specific topic
                                    it_sub_topic->second += 1;
                                }
                            } else {
                                // put msg in subs undelivered msg queue
                                msg->time = time_->t()+msg_delay;
                                sub->add_undelivered_msg(msg, is_stochastic_delay_);
                            }
                        }
                    }
                }
            }
        }
    }

    // Enforce queue sizes, if necessary
    for (auto &sub_kv : subs) {
        // For all publisher devices with topic name
        for (NetworkDevicePtr &sub : sub_kv.second) {
            sub->enforce_queue_size();
        }
    }

    if (write_csv_) {
        CSV::Pairs pairs;
        pairs.push_back(std::make_pair("t", time_->t()));

        for (auto &kv : pub_counts_) {
            std::string header = kv.first + "_Pub_Count";
            pairs.push_back(std::make_pair(header, kv.second));
        }

        for (auto &kv : sub_counts_) {
            std::string header = kv.first + "_Sub_Count";
            pairs.push_back(std::make_pair(header, kv.second));
        }
        csv_.append(pairs);
    }

    return true;
}


bool Network::is_reachable(const scrimmage::EntityPluginPtr &pub_plugin,
                           const scrimmage::EntityPluginPtr &sub_plugin) {
    return false;
}

bool Network::is_successful_transmission(const scrimmage::EntityPluginPtr &pub_plugin,
                                         const scrimmage::EntityPluginPtr &sub_plugin) {
    return false;
}

double Network::get_transmission_delay() {
    return comm_delay_;
}

std::string Network::type() {
    return std::string("Network");
}

void Network::set_rtree(const RTreePtr &rtree) {
    rtree_ = rtree;
}

// cppcheck-suppress passedByValue
void Network::set_random(RandomPtr random) {
    random_ = random;
}

void Network::close(double t) {
    rtree_ = nullptr;
    random_ = nullptr;
    mp_ = nullptr;
}

} // namespace scrimmage
