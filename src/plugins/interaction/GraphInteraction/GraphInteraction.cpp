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

#include <scrimmage/plugins/interaction/GraphInteraction/GraphInteraction.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
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

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::GraphInteraction,
                GraphInteraction_plugin)

namespace scrimmage {
namespace interaction {

GraphInteraction::GraphInteraction() {
}

std::ifstream& GotoLine(std::ifstream& file, unsigned int num) {
    file.seekg(std::ios::beg);
    for (unsigned int i = 0; i < num - 1; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return file;
}

// Uses graph file to draw graph
// https://github.com/AndGem/OsmToRoadGraph
bool GraphInteraction::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    pub_graph_ = advertise("GlobalNetwork", "Graph");
    std::string default_file_name = "default";
    std::string graph_file_name = sc::get<std::string>("graph_file", plugin_params, default_file_name);
    std::string labels_file_name = sc::get<std::string>("labels_file", plugin_params, default_file_name);
    vis_graph_ = sc::get<bool>("visualize_graph", plugin_params, true);
    id_ = sc::get<int>("id", plugin_params, 1);
    if (graph_file_name == default_file_name)
        return true;

    std::ifstream graph_file(graph_file_name);
    std::ifstream names_file(labels_file_name);

    bool found_labels = true;
    if (!names_file.good()) {
        found_labels = false;
        std::cout << labels_file_name << " not found. Resuming with default labels" << std::endl;
    }

    if (graph_file.is_open()) {
        std::string line;
        // Skip header until the node and edge count
        while (std::getline(graph_file, line)) {
            if (line[0] != '#') { // Line is node count
                std::getline(graph_file, line); // Line is edge count
                break;
            }
        }

        // Read the nodes and edges
        std::map<int, Eigen::Vector3d> nodes;
        auto graph_msg = std::make_shared<sc::Message<sm::Graph>>();
        graph_msg->data.set_id(id_);
        unsigned int edge_counter = 0;
        while (std::getline(graph_file, line)) {
            std::vector<std::string> words;
            boost::split(words, line, [](char c) { return c == ' '; });

            bool is_node = words.size() == 3;
            if (is_node) {
                double x, y, z;
                int id = std::stoi(words[0]);
                double latitude = std::stod(words[1]), longitude = std::stod(words[2]);
                parent_->projection()->Forward(latitude, longitude, 0.0, x, y, z);
                nodes[id] = Eigen::Vector3d(x, y, z);

                auto node_ptr = graph_msg->data.add_nodes();
                node_ptr->set_id(id);
                sc::set(node_ptr->mutable_point(), nodes[id]);
            } else { // is edge
                edge_counter++;
                int id_start = std::stoi(words[0]), id_end = std::stoi(words[1]);
                double length = std::stod(words[2]);

                bool edge_nodes_exist = nodes.count(id_start) > 0 && nodes.count(id_end) > 0; // reality check
                if (edge_nodes_exist) {
                    auto edge_ptr = graph_msg->data.add_edges();
                    edge_ptr->set_start_node_id(id_start);
                    edge_ptr->set_end_node_id(id_end);
                    edge_ptr->set_weight(length);

                    if (found_labels) {
                        // Get Street Name
                        GotoLine(names_file, edge_counter);
                        std::string street_name;
                        std::getline(names_file, street_name);
                        edge_ptr->set_label(street_name);
                    }

                    // Visualize the Edges
                    if (vis_graph_) {
                        auto edge_shape = std::make_shared<scrimmage_proto::Shape>();
                        edge_shape->set_persistent(true);
                        edge_shape->set_opacity(1.0);
                        scrimmage::set(edge_shape->mutable_color(), 255, 0, 0);
                        scrimmage::set(edge_shape->mutable_line()->mutable_start(), nodes[id_start]);
                        scrimmage::set(edge_shape->mutable_line()->mutable_end(), nodes[id_end]);
                        draw_shape(edge_shape);
                    }
                }
            }
        }
        pub_graph_->publish(graph_msg);

        // Visualize the Nodes
        if (vis_graph_) {
            auto node_shape = std::make_shared<scrimmage_proto::Shape>();
            node_shape->set_persistent(true);
            for (auto node : nodes) {
                scrimmage::set(node_shape->mutable_pointcloud()->add_point(), node.second[0], node.second[1], node.second[2]);
            }
            node_shape->mutable_pointcloud()->set_size(3);
            draw_shape(node_shape);
        }

        graph_file.close();
    } else {
        cout << "Unable to open file: " + graph_file_name;
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
} // namespace interaction
} // namespace scrimmage
