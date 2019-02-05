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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GRAPHVIZFSM_GRAPHVIZFSM_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GRAPHVIZFSM_GRAPHVIZFSM_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>

namespace scrimmage {
namespace autonomy {
class GraphvizFSM : public scrimmage::Autonomy {
 public:
    struct DotVertex {
        std::string name; //, label, shape;
    };

    struct label_t {
        typedef boost::edge_property_tag kind;
    };

    typedef boost::property<label_t, std::string> EdgeProperty;

    typedef boost::property<boost::graph_name_t, std::string> graph_p;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, DotVertex, EdgeProperty, graph_p, boost::listS> graph_t;

    GraphvizFSM();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    graph_t fsm_graph_;
    boost::graph_traits<graph_t>::vertex_descriptor current_state_;
    scrimmage::PublisherPtr state_pub_;

    boost::property_map<graph_t, label_t>::type label_;

    void update_state_info(
        boost::graph_traits<graph_t>::vertex_descriptor current_state,
        boost::graph_traits<graph_t>::vertex_descriptor next_state);

    bool print_current_state_ = false;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GRAPHVIZFSM_GRAPHVIZFSM_H_
