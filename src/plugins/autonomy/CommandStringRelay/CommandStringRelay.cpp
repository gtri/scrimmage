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

#include <scrimmage/plugins/autonomy/CommandStringRelay/CommandStringRelay.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/msgs/Command.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::CommandStringRelay,
                CommandStringRelay_plugin)

namespace scrimmage {
namespace autonomy {

CommandStringRelay::CommandStringRelay() {
}

void CommandStringRelay::init(std::map<std::string, std::string> &params) {
    std::string relays_str = sc::get<std::string>("relays", params, "");
    std::vector<std::vector<std::string>> vecs_of_vecs;
    sc::get_vec_of_vecs(relays_str, vecs_of_vecs, " ");
    for (std::vector<std::string> vecs : vecs_of_vecs) {
        if (vecs.size() < 3) {
            std::cout << "Invalid relay found." << std::endl;
            continue;
        }

        std::string original_network = vecs[0];
        std::string topic = vecs[1];
        std::string relay_to_network = vecs[2];
        std::string original_topic_str = "/" + std::to_string(parent_->id().id()) + "/" + topic;
        pubs_[topic] = advertise(relay_to_network, topic);

        auto cb = [&] (scrimmage::MessagePtr<scrimmage_msgs::CommandString> msg) {
            std::vector<std::string> tokens;
            boost::split(tokens, msg->data.topic(), boost::is_any_of("/"));

            if (tokens.size() == 3) {
                sc::PublisherPtr pub;
                auto it = pubs_.find(tokens[2]);
                if (it != pubs_.end()) {
                    auto relay_msg = std::make_shared<scrimmage::Message<std::string>>();
                    relay_msg->data = msg->data.value();
                    it->second->publish(relay_msg);
                } else {
                    cout << "CommandStringRelay: Failed to find publisher" << endl;
                }
            } else {
                cout << "CommandStrinRelay: Ignoring message" << endl;
            }
        };
        subscribe<scrimmage_msgs::CommandString>(original_network, original_topic_str, cb);
    }
}

bool CommandStringRelay::step_autonomy(double t, double dt) {
    return true;
}
} // namespace autonomy
} // namespace scrimmage
