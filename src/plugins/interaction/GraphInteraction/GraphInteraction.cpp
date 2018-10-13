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

#include <scrimmage/plugins/interaction/GraphInteraction/GraphUtils.h>
#include <scrimmage/plugins/interaction/GraphInteraction/GraphInteraction.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/msgs/Graph.pb.h>

#include <memory>
#include <limits>
#include <iostream>
#include <fstream>

#include <GeographicLib/LocalCartesian.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphml.hpp>

using std::cout;
using std::endl;
using std::stoi;


namespace fs = ::boost::filesystem;
namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::GraphInteraction,
                GraphInteraction_plugin)

namespace scrimmage {
namespace interaction {

GraphInteraction::GraphInteraction() {
}

// Uses graph file to draw graph
bool GraphInteraction::init(std::map<std::string, std::string> &mission_params,
        std::map<std::string, std::string> &plugin_params) {
    pub_graph_ = advertise("GlobalNetwork", "Graph");
    std::string default_file_name = "default";
    std::string graph_file_name =
        sc::get<std::string>("graph_file", plugin_params, default_file_name);

    // If the data tag has been set for this plugin in the  mission file, get
    // the graph and label filenames from that instead of using those above.
    std::map<std::string, std::string> data_params;
    if (sc::parse_autonomy_data(plugin_params, data_params)) {
        FileSearch file_search;
        graph_file_name = sc::get<std::string>("graph_file", data_params,
                default_file_name);
        if (graph_file_name != default_file_name) {
            std::string graph_ext =
                fs::path(graph_file_name).extension().string();
            file_search.find_file(graph_file_name, graph_ext,
                    "SCRIMMAGE_DATA_PATH", graph_file_name);
        }
    }

    vis_graph_ = sc::get<bool>("visualize_graph", plugin_params, true);
    id_ = sc::get<int>("id", plugin_params, 1);
    if (graph_file_name == default_file_name)
        return true;

    std::ifstream graph_file(graph_file_name);

    boost::dynamic_properties dp(boost::ignore_other_properties);
    dp.property("osmid", boost::get(&VertexProperties::osmid, g_));
    dp.property("x", boost::get(&VertexProperties::x, g_));
    dp.property("y", boost::get(&VertexProperties::y, g_));

    dp.property("geometry", boost::get(&EdgeProperties::geometry, g_));
    dp.property("length", boost::get(&EdgeProperties::length, g_));
    dp.property("osmid", boost::get(&EdgeProperties::osmid, g_));
    dp.property("name", boost::get(&EdgeProperties::name, g_));

    if (graph_file.is_open()) {
        boost::read_graphml(graph_file, g_, dp);
        cout << "read graph file" << endl;
        graph_file.close();
    }

    std::map<int64_t, Eigen::Vector3d> nodes;
    std::map<int64_t, int64_t> boost_vert_to_osmid;

    auto graph_msg = std::make_shared<sc::Message<sm::Graph>>();
    graph_msg->data.set_id(id_);
    for (auto vs = boost::vertices(g_); vs.first != vs.second; ++vs.first) {
        // double longitude = boost::get(&VertexProperties::x, g_, *vs.first);
        // double latitude = boost::get(&VertexProperties::y, g_, *vs.first);
        // int64_t this_osmid =
        //        boost::get(&VertexProperties::osmid, g_, *vs.first);
        // NOTE: the above three lines are equivalent to the below three lines
        double longitude = g_[*vs.first].x;
        double latitude = g_[*vs.first].y;
        double this_osmid = g_[*vs.first].osmid;
        boost_vert_to_osmid[*vs.first] = this_osmid;

        double x, y, z;
        parent_->projection()->Forward(latitude, longitude, 0.0, x, y, z);
        nodes[this_osmid] = Eigen::Vector3d(x, y, z);
        auto node_ptr = graph_msg->data.add_nodes();
        node_ptr->set_id(this_osmid);
        sc::set(node_ptr->mutable_point(), nodes[this_osmid]);
    }

    for (auto es = boost::edges(g_); es.first != es.second; ++es.first) {
        int64_t id_start = boost_vert_to_osmid[boost::source(*es.first, g_)];
        int64_t id_end = boost_vert_to_osmid[boost::target(*es.first, g_)];

        auto edge_ptr = graph_msg->data.add_edges();
        edge_ptr->set_start_node_id(id_start);
        edge_ptr->set_end_node_id(id_end);
        edge_ptr->set_weight(g_[*es.first].length);
        edge_ptr->set_label(g_[*es.first].name);
    }
    pub_graph_->publish(graph_msg);

    if (vis_graph_) {
        auto node_idx_to_pos = nodes_idxs_to_pos_map(graph_msg->data);
        auto ptr = shared_from_this();
        bool draw_node_labels = sc::get<bool>("draw_node_labels", plugin_params, true);
        auto draw = draw_node_labels ? DrawNodeLabels::YES : DrawNodeLabels::NO;
        draw_graph(graph_msg->data, node_idx_to_pos, draw, ptr);
    }

    return true;
}

bool GraphInteraction::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (ents.empty()) {
        return true;
    }

    return true;
}

}  // namespace interaction
}  // namespace scrimmage
