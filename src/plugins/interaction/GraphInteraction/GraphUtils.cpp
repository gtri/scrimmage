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

#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/msgs/Graph.pb.h>

#include <iomanip>
#include <sstream>

#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/transform.hpp>

namespace ba = boost::adaptors;
namespace br = boost::range;
namespace sp = scrimmage_proto;

namespace scrimmage {
namespace interaction {
void draw_graph(
        scrimmage_msgs::Graph &graph,
        const std::unordered_map<uint64_t, scrimmage_proto::Vector3d> &node_idx_to_pos,
        DrawNodeLabels draw_node_labels,
        std::shared_ptr<Plugin> plugin) {

    const std::vector<int> black {0, 0, 0};
    const std::vector<int> white {255, 255, 255};
    const std::vector<int> blue {0, 0, 255};

    for (auto &e : graph.edges()) {
        auto edge_shape = std::make_shared<sp::Shape>();
        set(edge_shape->mutable_color(), black);
        edge_shape->set_opacity(1.0);
        edge_shape->set_persistent(true);

        auto &p1 = node_idx_to_pos.at(e.start_node_id());
        auto &p2 = node_idx_to_pos.at(e.end_node_id());

        set(edge_shape->mutable_line()->mutable_start(), p1.x(), p1.y(), p1.z());
        set(edge_shape->mutable_line()->mutable_end(), p2.x(), p2.y(), p2.z());
        plugin->draw_shape(edge_shape);
    }

    auto node_shape = std::make_shared<sp::Shape>();
    node_shape->set_persistent(true);
    auto to_pt = [&](auto &n) {return n.point();};
    for (const auto &pt : graph.nodes() | ba::transformed(to_pt)) {
        set(node_shape->mutable_pointcloud()->add_point(), pt.x(), pt.y(), pt.z());
        set(node_shape->mutable_pointcloud()->add_color(), blue);
    }
    node_shape->mutable_pointcloud()->set_size(6);
    plugin->draw_shape(node_shape);

    for (const auto &node : graph.nodes()) {
        auto text_shape = std::make_shared<sp::Shape>();
        text_shape->set_persistent(true);
        text_shape->set_opacity(1.0);
        set(text_shape->mutable_color(), white);

        if (draw_node_labels == DrawNodeLabels::YES) {
            Eigen::Vector3d pos(node.point().x(), node.point().y(), node.point().z());
            pos(0) += 5;
            set(text_shape->mutable_text()->mutable_center(), pos);
            text_shape->mutable_text()->set_scale(20);

            text_shape->mutable_text()->set_text(std::to_string(node.id()));
            plugin->draw_shape(text_shape);
        }
    }
}

std::unordered_map<uint64_t, scrimmage_proto::Vector3d> nodes_idxs_to_pos_map(
        const scrimmage_msgs::Graph &graph) {

    auto to_pos = [&](auto &node) {return std::make_pair(node.id(), node.point());};
    std::unordered_map<uint64_t, scrimmage_proto::Vector3d> out;
    br::transform(graph.nodes(), std::inserter(out, out.begin()), to_pos);
    return out;
}
} // namespace interaction
} // namespace scrimmage
