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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRPCCOMMANDSTRING_GRPCCOMMANDSTRING_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRPCCOMMANDSTRING_GRPCCOMMANDSTRING_H_

#include <scrimmage/simcontrol/EntityInteraction.h>

#include <scrimmage/msgs/Command.pb.h>

#include <map>
#include <list>
#include <string>
#include <queue>
#include <thread> // NOLINT
#include <mutex> // NOLINT

namespace sc = scrimmage;

namespace scrimmage {
namespace interaction {

class GRPCCommandString : public scrimmage::EntityInteraction {
 public:
    GRPCCommandString();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                 double t, double dt) override;
    void run_server();

    void push_msg(const scrimmage_msgs::CommandString &msg);

 protected:
    std::string ip_ = "localhost";
    int port_ = 60000;
    std::thread network_thread_;

    std::mutex msgs_mutex_;
    std::queue<scrimmage_msgs::CommandString> msgs_;

    std::map<std::string, std::map<std::string, sc::PublisherPtr>> pubs_;

 private:
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRPCCOMMANDSTRING_GRPCCOMMANDSTRING_H_
