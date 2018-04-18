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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <iostream>
#include <iomanip>

#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/range/adaptor/map.hpp>

namespace br = boost::range;
namespace ba = boost::adaptors;

namespace scrimmage {

bool create_ent_inters(const SimUtilsInfo &info,
                       RandomPtr random,
                       std::list<scrimmage_proto::ShapePtr> &shapes,
                       std::list<EntityInteractionPtr> &ent_inters) {

    for (std::string ent_inter_name : info.mp->entity_interactions()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            info.mp->attributes()[ent_inter_name];
        EntityInteractionPtr ent_inter =
            std::dynamic_pointer_cast<EntityInteraction>(
                info.plugin_manager->make_plugin(
                    "scrimmage::EntityInteraction",
                    ent_inter_name, *info.file_search,
                    config_parse, overrides));

        if (ent_inter == nullptr) {
            std::cout << "Failed to load entity interaction plugin: "
                 << ent_inter_name << std::endl;
            return false;
        }

        ent_inter->set_random(random);
        ent_inter->set_mission_parse(info.mp);
        ent_inter->set_projection(info.mp->projection());
        ent_inter->set_pubsub(info.pubsub);
        ent_inter->set_time(info.time);
        ent_inter->set_id_to_team_map(info.id_to_team_map);
        ent_inter->set_id_to_ent_map(info.id_to_ent_map);
        ent_inter->set_name(ent_inter_name);
        ent_inter->init(info.mp->params(), config_parse.params());
        ent_inter->parent()->rtree() = info.rtree;

        // Get shapes from plugin
        shapes.insert(
            shapes.end(), ent_inter->shapes().begin(), ent_inter->shapes().end());
        ent_inter->shapes().clear();

        ent_inters.push_back(ent_inter);
    }

    return true;
}

bool create_metrics(const SimUtilsInfo &info, std::list<MetricsPtr> &metrics_list) {

    for (std::string metrics_name : info.mp->metrics()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            info.mp->attributes()[metrics_name];
        MetricsPtr metrics =
            std::dynamic_pointer_cast<Metrics>(
                info.plugin_manager->make_plugin(
                    "scrimmage::Metrics", metrics_name,
                    *info.file_search, config_parse, overrides));

        if (metrics == nullptr) {
            std::cout << "Failed to load metrics: " << metrics_name << std::endl;
            return false;
        }

        metrics->set_id_to_team_map(info.id_to_team_map);
        metrics->set_id_to_ent_map(info.id_to_ent_map);
        metrics->set_pubsub(info.pubsub);
        metrics->set_time(info.time);
        metrics->set_name(metrics_name);
        metrics->parent()->rtree() = info.rtree;
        metrics->init(config_parse.params());
        metrics_list.push_back(metrics);
    }

    return true;
}

void run_callbacks(PluginPtr plugin) {
    for (auto &sub : plugin->subs()) {
        for (auto msg : sub->pop_msgs<MessageBase>()) {
            sub->accept(msg);
        }
    }
}

bool verify_io_connection(VariableIO &output, VariableIO &input) {
    std::vector<std::string> mismatched_keys;
    br::set_difference(
        input.declared_input_variables(),
        output.declared_output_variables(),
        std::back_inserter(mismatched_keys));

    return mismatched_keys.empty();
}

void print_io_error(const std::string &in_name, const std::string &out_name, VariableIO &v) {
    auto keys = v.input_variable_index() | ba::map_keys;
    std::cout << "First, place the following in its initializer: " << std::endl;
    for (const std::string &key : keys) {
        std::cout << "    " << key << "_idx_ = vars_.declare("
            << std::quoted(key) << ", VariableIO::Direction::Out);"
            << std::endl;
    }

    std::cout << "Second, place the following in its step function: " << std::endl;
    for (const std::string &key : keys) {
        std::cout << "    vars_.output(" << key << "_idx_, value_to_output);" << std::endl;
    }
    std::cout << "where value_to_output is what you want " << in_name
        << " to receive as its input." << std::endl;

    std::cout << "Third, place following in the class declaration: " << std::endl;
    for (const std::string &key : keys) {
        std::cout << "    uint8_t " << key << "_idx_ = 0;" << std::endl;
    }
}
} // namespace scrimmage
