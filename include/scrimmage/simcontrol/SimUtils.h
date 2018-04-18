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

#ifndef INCLUDE_SCRIMMAGE_SIMCONTROL_SIMUTILS_H_
#define INCLUDE_SCRIMMAGE_SIMCONTROL_SIMUTILS_H_

#include <scrimmage/fwd_decl.h>

#include <memory>
#include <unordered_map>
#include <list>
#include <string>

namespace scrimmage {

struct SimUtilsInfo {
    MissionParsePtr mp;
    PluginManagerPtr plugin_manager;
    FileSearchPtr file_search;
    RTreePtr rtree;
    PubSubPtr pubsub;
    TimePtr time;
    std::shared_ptr<std::unordered_map<int, int>> id_to_team_map;
    std::shared_ptr<std::unordered_map<int, EntityPtr>> id_to_ent_map;
};

bool create_ent_inters(const SimUtilsInfo &info,
                       RandomPtr random,
                       std::list<scrimmage_proto::ShapePtr> &shapes,
                       std::list<EntityInteractionPtr> &ent_inters);

bool create_metrics(const SimUtilsInfo &info, std::list<MetricsPtr> &metrics_list);

void run_callbacks(PluginPtr plugin);

void print_io_error(const std::string &in_name, const std::string &out_name, VariableIO &v);

bool verify_io_connection(VariableIO &output_plugin, VariableIO &input_plugin);

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_SIMCONTROL_SIMUTILS_H_
