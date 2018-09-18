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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPH_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPH_H_

#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/msgs/Graph.pb.h>
#include <scrimmage/pubsub/Message.h>

#include <Eigen/Dense>

#include <vector>
#include <map>
#include <list>
#include <string>
#include <iostream>


namespace sm = scrimmage_msgs;
namespace scrimmage {

class Node {
 public:
    Node() {}
    Node(int id, Eigen::Vector3d point) : id(id), point(point) {}

    int id;
    Eigen::Vector3d point;
};

class Edge {
 public:
    Edge(int start_id, int end_id, double weight, std::string label) :
        start_node_id(start_id), end_node_id(end_id), weight(weight), label(label) {}
    Edge(Node node_a, Node node_b, std::string label) {
        start_node_id = node_a.id;
        end_node_id = node_b.id;
        weight = (node_a.point - node_b.point).norm();
        label = this->label;
    }
    int start_node_id, end_node_id;
    double weight;
    std::string label;
};



class Graph {
 public:
    int id;
    std::map<int, Node> nodes; // node IDs map to Nodes
    std::vector<Edge> edges;

    Graph() {}
    explicit Graph(scrimmage::MessagePtr<sm::Graph> msg) {
        id = msg->data.id();
        for (auto elem : msg->data.nodes()) {
            Node node(elem.id(), eigen(elem.point()));
            nodes[node.id] = node;
        }
        for (auto elem : msg->data.edges()) {
            Edge edge(elem.start_node_id(), elem.end_node_id(), elem.weight(), elem.label());
            edges.push_back(edge);
        }
    }

 private:
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPH_H_
