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

#ifndef INCLUDE_SCRIMMAGE_ENTITY_ENTITYPLUGINHELPER_H_
#define INCLUDE_SCRIMMAGE_ENTITY_ENTITYPLUGINHELPER_H_

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/optional.hpp>

#include "scrimmage/autonomy/Autonomy.h"
#include "scrimmage/fwd_decl.h"
#include "scrimmage/log/Logger.h"
#include "scrimmage/motion/Controller.h"
#include "scrimmage/parse/ConfigParse.h"
#include "scrimmage/plugin_manager/PluginManager.h"

namespace scrimmage {

template <class T>
boost::optional<std::shared_ptr<T>> make_autonomy(
    const std::string& autonomy_name,
    PluginManagerPtr plugin_manager,
    std::map<std::string, std::string>& overrides,
    EntityPtr parent,
    StatePtr state,
    std::shared_ptr<std::unordered_map<int, int>>& id_to_team_map,
    std::shared_ptr<std::unordered_map<int, EntityPtr>>& id_to_ent_map,
    std::shared_ptr<GeographicLib::LocalCartesian> proj,
    ContactMapPtr& contacts,
    FileSearchPtr& file_search,
    RTreePtr& rtree,
    PubSubPtr& pubsub,
    const std::shared_ptr<const Time>& time,
    const ParameterServerPtr& param_server,
    const std::set<std::string>& plugin_tags = {},
    std::function<void(std::map<std::string, std::string>&)> param_override_func =
        [](std::map<std::string, std::string>& params) {},
    std::vector<ControllerPtr> controllers = {},
    const int& debug_level = 0) {

    ConfigParse config_parse;
    PluginStatus<T> status = plugin_manager->make_plugin<T>(
        "scrimmage::Autonomy",
        autonomy_name,
        *file_search,
        config_parse,
        overrides,
        plugin_tags);
    if (status.status == PluginStatus<T>::cast_failed) {
        LOG_ERROR("Failed to open autonomy plugin: " << autonomy_name);
    } else if (status.status == PluginStatus<T>::parse_failed) {
        LOG_ERROR("Failed to parse autonomy plugin config: " << autonomy_name);
    } else if (status.status == PluginStatus<T>::loaded) {
        std::shared_ptr<T> autonomy = status.plugin;
        // Connect the autonomy to the first controller
        if (not controllers.empty()) {
            connect(autonomy->vars(), controllers.front()->vars());
        }
        autonomy->set_rtree(rtree);
        autonomy->set_parent(parent);
        autonomy->set_projection(proj);
        autonomy->set_pubsub(pubsub);
        autonomy->set_time(time);
        autonomy->set_id_to_team_map(id_to_team_map);
        autonomy->set_id_to_ent_map(id_to_ent_map);
        autonomy->set_param_server(param_server);
        autonomy->set_state(state);
        autonomy->set_contacts(contacts);
        autonomy->set_is_controlling(true);
        autonomy->set_name(autonomy_name);
        param_override_func(config_parse.params());

        if (debug_level > 1) {
            LOG_INFO("--------------------------------");
            LOG_INFO("Autonomy plugin params: " << autonomy_name);
            LOG_INFO(config_parse);
        }
        try {
            autonomy->init(config_parse.params());
        } catch (const std::exception& e) {
            LOG_ERROR("Autonomy plugin '" << autonomy_name << "' threw exception during init: " << e.what());
            return boost::none;
        } catch (...) {
            LOG_ERROR("Autonomy plugin '" << autonomy_name << "' threw unknown exception during init");
            return boost::none;
        }

        // get loop rate from plugin's params
        auto it_loop_rate = config_parse.params().find("loop_rate");
        if (it_loop_rate != config_parse.params().end()) {
            try {
                const double loop_rate = std::stod(it_loop_rate->second);
                autonomy->set_loop_rate(loop_rate);
            } catch (const std::exception& e) {
                LOG_ERROR("Autonomy plugin '" << autonomy_name << "': invalid loop_rate '" << it_loop_rate->second << "': " << e.what());
                return boost::none;
            }
        }
        return boost::optional<std::shared_ptr<T>>{autonomy};
    }
    return boost::none;
}

}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_ENTITY_ENTITYPLUGINHELPER_H_
