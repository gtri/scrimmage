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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHUTILS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHUTILS_H_

#include <memory>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <utility>

#include <boost/functional/hash.hpp>

namespace scrimmage_msgs {
class Graph;
} // namespace scrimmage_msgs

namespace scrimmage_proto {
class Shape;
} // namespace scrimmage_proto

namespace scrimmage_proto {
class Vector3d;
} // namespace scrimmage_proto

namespace scrimmage {

class EntityPlugin;
using EntityPluginPtr = std::shared_ptr<EntityPlugin>;

namespace interaction {

enum class DrawNodeLabels {NO, YES};
using NodePair = std::pair<uint64_t, uint64_t>;
using NodePairColorMap = std::unordered_map<NodePair, std::vector<int>, boost::hash<NodePair>>;
using NodePairShapeMap = std::unordered_map<NodePair, std::shared_ptr<scrimmage_proto::Shape>, boost::hash<NodePair>>;
using NodeShapeMap = std::unordered_map<uint64_t, std::shared_ptr<scrimmage_proto::Shape>>;

std::tuple<std::shared_ptr<scrimmage_proto::Shape>, NodePairShapeMap, NodeShapeMap>
draw_graph(
    scrimmage_msgs::Graph &graph,
    const std::unordered_map<uint64_t, scrimmage_proto::Vector3d> &node_idx_to_pos,
    DrawNodeLabels draw_node_labels,
    EntityPluginPtr plugin);

std::unordered_map<uint64_t, scrimmage_proto::Vector3d> nodes_idxs_to_pos_map(
    const scrimmage_msgs::Graph &graph);
} // namespace interaction
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHUTILS_H_
