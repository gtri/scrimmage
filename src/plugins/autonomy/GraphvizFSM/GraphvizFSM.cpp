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

#include <scrimmage/plugins/autonomy/GraphvizFSM/GraphvizFSM.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/string/replace.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::GraphvizFSM,
                GraphvizFSM_plugin)

namespace scrimmage {
namespace autonomy {

GraphvizFSM::GraphvizFSM() {
}

void GraphvizFSM::init(std::map<std::string, std::string> &params) {
    print_current_state_ = sc::get<bool>("print_current_state", params, print_current_state_);
    std::string graph_str = sc::get<std::string>("graphviz_fsm", params, "");

    // Replace any ' with " characters. This allows the graphviz_fsm string to
    // include single quotes instead of having to use &quot;
    boost::replace_all(graph_str, "'", "\"");

    boost::dynamic_properties dp(boost::ignore_other_properties);

    dp.property("node_id", boost::get(&DotVertex::name,  fsm_graph_));

    label_ = get(label_t(), fsm_graph_);
    dp.property("label", label_);

    if (!boost::read_graphviz(graph_str, fsm_graph_, dp)) {
        cout << "Failed to parse graphviz fms" << endl;
        return;
    }

    bool create_fsm_pdf = sc::get<bool>("generate_fsm_pdf", params, false);
    if (create_fsm_pdf) {
        std::ofstream gv_file("./fsm.gv");
        gv_file << graph_str;
        gv_file.close();

        int result = system("dot -Tpdf fsm.gv -o fsm.pdf");
        if (result != 0) {
            cout << "Error during FSM pdf creation. Make sure the dot program "
                 << "is installed." << endl;
        }
    }

    std::string qi_name = sc::get<std::string>("initial_node_name", params, "qi");

    // Find the initial state
    typedef boost::graph_traits<graph_t>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = boost::vertices(fsm_graph_); vp.first != vp.second; ++vp.first) {
        boost::graph_traits<graph_t>::vertex_descriptor v = *vp.first;
        if (qi_name == fsm_graph_[v].name) {
            current_state_ = v;
        }
    }

    std::string state_topic = sc::get<std::string>("state_topic_name", params, "State");
    std::string event_topic = sc::get<std::string>("event_topic_name", params, "Event");
    std::string network_name = sc::get<std::string>("network_name", params, "LocalNetwork");

    // Setup publisher on the state topic
    state_pub_ = advertise(network_name, state_topic);

    // Subscribe to event topic
    auto event_callback = [&] (scrimmage::MessagePtr<std::string> msg) {
        // If the event received is a trigger for the current state, transition
        // to the next state.
        typename boost::graph_traits<graph_t>::out_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(current_state_, fsm_graph_); ei != ei_end; ++ei) {
            if (label_(*ei) == msg->data) {
                this->update_state_info(current_state_,
                                        boost::target(*ei, fsm_graph_));
            }
        }
    };
    subscribe<std::string>(network_name, event_topic, event_callback);

    return;
}

bool GraphvizFSM::step_autonomy(double t, double dt) {
    // If the current state only has one out edge (degree == 1) and doesn't
    // have a trigger label to its next state, immediately move to that next
    // state since this is just a designator for the "initial" state.
    if (boost::out_degree(current_state_, fsm_graph_) == 1) {
        typename boost::graph_traits<graph_t>::out_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(current_state_, fsm_graph_); ei != ei_end; ++ei) {
            if (label_(*ei) == std::string("")) {
                this->update_state_info(current_state_,
                                        boost::target(*ei, fsm_graph_));
            }
        }
    }
    return true;
}

void GraphvizFSM::update_state_info(
    boost::graph_traits<graph_t>::vertex_descriptor current_state,
    boost::graph_traits<graph_t>::vertex_descriptor next_state) {

    current_state_ = next_state;

    if (print_current_state_) {
        cout << "Current State: " << fsm_graph_[current_state_].name << endl;
    }
    // Publish the state message
    auto msg = std::make_shared<sc::Message<std::string>>();
    msg->data = fsm_graph_[current_state_].name;
    state_pub_->publish(msg);
}

} // namespace autonomy
} // namespace scrimmage
