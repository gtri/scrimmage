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

#include <scrimmage/common/FileSearch.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <signal.h>
#include <cstdlib>

#include <iostream>

using std::cout;
using std::endl;

namespace scrimmage {

bool create_ent_inters(const SimUtilsInfo &info,
                       ContactMapPtr contacts,
                       std::list<scrimmage_proto::ShapePtr> &shapes,
                       std::list<EntityInteractionPtr> &ent_inters,
                       const GlobalServicePtr global_services,
                       const std::set<std::string> &plugin_tags,
                       std::function<void(std::map<std::string, std::string>&)> param_override_func,
                       const int& debug_level) {
    for (std::string ent_inter_name : info.mp->entity_interactions()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            info.mp->attributes()[ent_inter_name];

        PluginStatus<EntityInteraction> status =
            info.plugin_manager->make_plugin<EntityInteraction>(
                "scrimmage::EntityInteraction",
                ent_inter_name, *info.file_search,
                config_parse, overrides, plugin_tags);

        if (status.status == PluginStatus<EntityInteraction>::cast_failed) {
            cout << "Failed to load entity interaction plugin: "
                      << ent_inter_name << endl;
        } else if (status.status == PluginStatus<EntityInteraction>::loaded) {
            EntityInteractionPtr ent_inter = status.plugin;

            // If the name was overridden, use the override.
            std::string name = get<std::string>("name", config_parse.params(),
                                                ent_inter_name);
            // Parent specific members
            ent_inter->parent()->set_random(info.random);
            ent_inter->parent()->set_mp(info.mp);
            ent_inter->parent()->set_projection(info.mp->projection());
            ent_inter->parent()->rtree() = info.rtree;
            ent_inter->parent()->contacts() = contacts;
            ent_inter->parent()->set_global_services(global_services);

            // Plugin specific members
            ent_inter->set_name(name);
            ent_inter->set_pubsub(info.pubsub);
            ent_inter->set_time(info.time);
            ent_inter->set_param_server(info.param_server);
            ent_inter->set_id_to_team_map(info.id_to_team_map);
            ent_inter->set_id_to_ent_map(info.id_to_ent_map);

            param_override_func(config_parse.params());

            if (debug_level > 1) {
                cout << "--------------------------------" << endl;
                cout << "Entity interaction plugin params: " << name << endl;
                cout << config_parse;
            }
            ent_inter->init(info.mp->params(), config_parse.params());

            // Get shapes from plugin
            shapes.insert(
                shapes.end(), ent_inter->shapes().begin(), ent_inter->shapes().end());
            ent_inter->shapes().clear();

            ent_inters.push_back(ent_inter);
        }
    }

    return true;
}

bool create_metrics(const SimUtilsInfo &info,
                    ContactMapPtr contacts,
                    std::list<MetricsPtr> &metrics_list,
                    const std::set<std::string> &plugin_tags,
                    std::function<void(std::map<std::string, std::string>&)> param_override_func,
                    const int& debug_level) {

    for (std::string metrics_name : info.mp->metrics()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            info.mp->attributes()[metrics_name];

        PluginStatus<Metrics> status =
            info.plugin_manager->make_plugin<Metrics>(
                "scrimmage::Metrics", metrics_name,
                *info.file_search, config_parse, overrides,
                plugin_tags);

        if (status.status == PluginStatus<Metrics>::cast_failed) {
            cout << "Failed to load metrics: " << metrics_name << endl;
            return false;
        } else if (status.status == PluginStatus<Metrics>::loaded) {
            MetricsPtr metrics = status.plugin;
            // Parent specific members
            metrics->parent()->set_random(info.random);
            metrics->parent()->set_mp(info.mp);
            metrics->parent()->projection() = info.mp->projection();
            metrics->parent()->rtree() = info.rtree;
            metrics->parent()->contacts() = contacts;

            // Plugin specific members
            metrics->set_name(metrics_name);
            metrics->set_pubsub(info.pubsub);
            metrics->set_time(info.time);
            metrics->set_param_server(info.param_server);
            metrics->set_id_to_team_map(info.id_to_team_map);
            metrics->set_id_to_ent_map(info.id_to_ent_map);

            param_override_func(config_parse.params());

            if (debug_level > 1) {
                cout << "--------------------------------" << endl;
                cout << "Metrics plugin params: " << metrics_name << endl;
                cout << config_parse;
            }
            metrics->init(config_parse.params());
            metrics_list.push_back(metrics);
        }
    }

    return true;
}

void run_callbacks(EntityPluginPtr plugin) {
    for (auto &sub : plugin->subs()) {
        for (auto msg : sub->pop_msgs<MessageBase>()) {
            sub->accept(msg);
        }
    }
}

namespace {
// https://stackoverflow.com/a/48164204
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
} // namespace

boost::optional<std::string> run_test(const std::string& mission,
                                      const bool& init_python,
                                      const bool& shutdown_python) {
    SimControl simcontrol;
    if (not simcontrol.init(mission, init_python)) {
        cout << "Failed to initialize SimControl." << endl;
        return boost::none;
    }

    // Handle kill signals
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    shutdown_handler = [&](int /*s*/){
        cout << endl << "Exiting gracefully" << endl;
        simcontrol.force_exit();
    };
    sa.sa_handler = signal_handler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    simcontrol.mp()->set_time_warp(-1);
    simcontrol.mp()->set_enable_gui(false);

    simcontrol.pause(false);
    if (not simcontrol.run()) {
        return boost::none;
    }

    // Extract the log directory
    auto out = boost::optional<std::string>{simcontrol.mp()->log_dir()};

    // Shutdown SimControl
    if (not simcontrol.shutdown(shutdown_python)) {
        return boost::none;
    }
    return out;
}

bool create_networks(const SimUtilsInfo &info, NetworkMap &networks,
                     const std::set<std::string> &plugin_tags,
                     std::function<void(std::map<std::string, std::string>&)> param_override_func,
                     const int& debug_level) {

    for (std::string network_name : info.mp->network_names()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            info.mp->attributes()[network_name];

        PluginStatus<Network> status =
            info.plugin_manager->make_plugin<Network>(
                "scrimmage::Network", network_name, *info.file_search,
                config_parse, overrides, plugin_tags);

        if (status.status == PluginStatus<Network>::cast_failed) {
            cout << "Failed to load network plugin: "
                << network_name << endl;
            return false;
        } else if (status.status == PluginStatus<Network>::loaded) {
            NetworkPtr network = status.plugin;
            // If the name was overridden, use the override.
            std::string name =
                get<std::string>("name", config_parse.params(), network_name);
            network->set_name(name);
            network->set_mission_parse(info.mp);
            network->set_time(info.time);
            network->set_param_server(info.param_server);
            network->set_pubsub(info.pubsub);
            network->set_random(info.random);
            network->set_rtree(info.rtree);
            network->set_id_to_team_map(info.id_to_team_map);
            network->set_id_to_ent_map(info.id_to_ent_map);

            // Seed the pubsub with network names
            info.pubsub->add_network_name(name);
            param_override_func(config_parse.params());

            if (debug_level > 1) {
                cout << "--------------------------------" << endl;
                cout << "Network plugin params: " << name << endl;
                cout << config_parse;
            }

            network->init(info.mp->params(), config_parse.params());
            networks[name] = network;
        }
    }
    return true;
}
} // namespace scrimmage
