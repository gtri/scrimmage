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
#include <grpc++/grpc++.h>

#include <scrimmage/plugins/interaction/GRPCCommandString/GRPCCommandString.h>
#include <scrimmage/plugins/interaction/GRPCCommandString/ScrimmageMsgServiceImpl.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Publisher.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::GRPCCommandString,
                GRPCCommandString_plugin)

namespace scrimmage {
namespace interaction {

GRPCCommandString::GRPCCommandString() {
}

bool GRPCCommandString::init(std::map<std::string, std::string> &mission_params,
                             std::map<std::string, std::string> &plugin_params) {
    ip_ = sc::get<std::string>("ip", plugin_params, ip_);
    port_ = sc::get<int>("port", plugin_params, port_);

    network_thread_ = std::thread(&GRPCCommandString::run_server, this);

    return true;
}


bool GRPCCommandString::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    // Check for new messages:
    msgs_mutex_.lock();
    while (msgs_.size() > 0) {
        scrimmage_msgs::CommandString msg = msgs_.front();
        msgs_.pop();

        // Does this publisher exist already?
        sc::PublisherPtr pub = nullptr;
        auto it_network = pubs_.find(msg.network());
        if (it_network != pubs_.end()) {
            auto it_topic = it_network->second.find(msg.topic());
            if (it_topic != it_network->second.end()) {
                pub = it_topic->second;
            }
        }
        // If the publisher doesn't exist yet, create it.
        if (pub == nullptr) {
            pub = advertise(msg.network(), msg.topic());
            pubs_[msg.network()][msg.topic()] = pub;
        }
        auto sc_msg = std::make_shared<scrimmage::Message<scrimmage_msgs::CommandString>>();
        sc_msg->data = msg;
        pub->publish(sc_msg);
    }
    msgs_mutex_.unlock();

    return true;
}

void GRPCCommandString::run_server() {
    std::string result = ip_ + ":" + std::to_string(port_);
    ScrimmageMsgServiceImpl service(shared_from_this());
    grpc::ServerBuilder builder;
    builder.AddListeningPort(result, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    std::cout << "GRPCCommandString listening on " << result << std::endl;
    server->Wait(); // this function blocks (should be in thread now)
}

void GRPCCommandString::push_msg(const scrimmage_msgs::CommandString &msg) {
    msgs_mutex_.lock();
    msgs_.push(msg);
    msgs_mutex_.unlock();
}

} // namespace interaction
} // namespace scrimmage
