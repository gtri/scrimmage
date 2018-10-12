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
 * A int64 description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHINTERACTION_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHINTERACTION_H_

#include <scrimmage/simcontrol/EntityInteraction.h>

#include <map>
#include <list>
#include <string>

#include <boost/graph/adjacency_list.hpp>

namespace scrimmage {

class Entity;
using EntityPtr = std::shared_ptr<Entity>;

class Publisher;
using PublisherPtr = std::shared_ptr<Publisher>;

namespace interaction {

class GraphInteraction : public scrimmage::EntityInteraction {
 public:
    GraphInteraction();
    bool init(std::map<std::string, std::string> &mission_params,
                std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                    double t, double dt) override;

 protected:
    struct GraphData {
        std::string Name;
    };

    struct VertexProperties {
        uint64_t osmid;
        double x;
        double y;
    };

    struct EdgeProperties {
        std::string geometry;
        std::string highway;
        double length;
        std::string osmid;
        std::string name;
    };

    typedef boost::adjacency_list<
        boost::vecS,  // edge storage
        boost::vecS,   // vertex storage
        boost::directedS,
        VertexProperties, EdgeProperties> Graph;
    Graph g_;

 private:
    bool vis_graph_ = true;
    PublisherPtr pub_graph_;
    int id_ = 1;
};
}  // namespace interaction
}  // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHINTERACTION_H_
