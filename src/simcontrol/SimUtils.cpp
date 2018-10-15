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
#include <iomanip>

#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/range/adaptor/map.hpp>

#if ENABLE_PYTHON_BINDINGS == 1
#include <pybind11/pybind11.h>
#endif

namespace br = boost::range;
namespace ba = boost::adaptors;

namespace scrimmage {

bool create_ent_inters(const SimUtilsInfo &info,
                       ContactMapPtr contacts,
                       std::list<scrimmage_proto::ShapePtr> &shapes,
                       std::list<EntityInteractionPtr> &ent_inters,
                       const std::set<std::string> &plugin_tags,
                       std::function<void(std::map<std::string, std::string>&)> param_override_func) {

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
            std::cout << "Failed to load entity interaction plugin: "
                      << ent_inter_name << std::endl;
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

            // Plugin specific members
            ent_inter->set_name(name);
            ent_inter->set_pubsub(info.pubsub);
            ent_inter->set_time(info.time);
            ent_inter->set_param_server(info.param_server);
            ent_inter->set_id_to_team_map(info.id_to_team_map);
            ent_inter->set_id_to_ent_map(info.id_to_ent_map);

            param_override_func(config_parse.params());
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
                    std::function<void(std::map<std::string, std::string>&)> param_override_func) {

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
            std::cout << "Failed to load metrics: " << metrics_name << std::endl;
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
            metrics->init(config_parse.params());
            metrics_list.push_back(metrics);
        }
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

void print_io_error(const std::string &in_name, VariableIO &v) {
    auto keys = v.input_variable_index() | ba::map_keys;

    std::cout << "First, include the VariableIO class in the cpp file: "
        << "#include <scrimmage/common/VariableIO.h>" << std::endl;

    std::cout << "Second, place the following in its initializer: " << std::endl;
    for (const std::string &key : keys) {
        std::cout << "    " << key << "_idx_ = vars_.declare("
            << std::quoted(key) << ", scrimmage::VariableIO::Direction::Out);"
            << std::endl;
    }

    std::cout << "Third, place the following in its step function: " << std::endl;
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

namespace {
// https://stackoverflow.com/a/48164204
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
} // namespace

boost::optional<std::string> run_test(std::string mission, bool init_python) {

    auto found_mission = FileSearch().find_mission(mission);
    if (!found_mission) {
        std::cout << "scrimmage::run_test could not find " << mission << std::endl;
        return boost::none;
    } else {
        SimControl simcontrol;

        // Handle kill signals
        struct sigaction sa;
        memset( &sa, 0, sizeof(sa) );
        shutdown_handler = [&](int /*s*/){
            std::cout << std::endl << "Exiting gracefully" << std::endl;
            simcontrol.force_exit();
        };
        sa.sa_handler = signal_handler;
        sigfillset(&sa.sa_mask);
        sigaction(SIGINT, &sa, NULL);
        sigaction(SIGTERM, &sa, NULL);

        auto mp = std::make_shared<MissionParse>();
        mp->set_time_warp(-1);
        mp->set_enable_gui(false);

        if (!mp->parse(*found_mission)) {
            std::cout << "Failed to parse file: " << *found_mission << std::endl;
            return boost::none;
        }

#if ENABLE_PYTHON_BINDINGS == 1
        if (init_python) Py_Initialize();
#endif
        auto log = preprocess_scrimmage(mp, simcontrol);
        if (log == nullptr) {
            return boost::none;
        }
        simcontrol.pause(false);
        simcontrol.run();

        auto out = postprocess_scrimmage(mp, simcontrol, log);

#if ENABLE_PYTHON_BINDINGS == 1
        if (init_python) Py_Finalize();
#endif
        return out;
    }
}

bool logging_logic(MissionParsePtr mp, std::string s) {
    std::string output_type = get("output_type", mp->params(), std::string("frames"));
    bool output_all = output_type.find("all") != std::string::npos;
    return output_all || output_type.find(s) != std::string::npos;
}

std::shared_ptr<Log> preprocess_scrimmage(
        MissionParsePtr mp,
        SimControl &simcontrol) {

    bool output_all = logging_logic(mp, "all");
    bool output_frames = logging_logic(mp, "frames");
    bool output_summary = logging_logic(mp, "summary");
    bool output_git = logging_logic(mp, "git_commits");
    bool output_mission = logging_logic(mp, "mission");
    bool output_seed = logging_logic(mp, "seed");
    bool output_nothing =
        !output_all && !output_frames && !output_summary &&
        !output_git && !output_mission && !output_seed;

    simcontrol.set_limited_verbosity(output_nothing);

    if (!output_nothing) {
        mp->create_log_dir();
    }

    auto log = setup_logging(mp);

    // Overwrite the seed if it's set
    simcontrol.set_log(log);

    InterfacePtr to_gui_interface = std::make_shared<Interface>();
    InterfacePtr from_gui_interface = std::make_shared<Interface>();
    to_gui_interface->set_log(log);
    from_gui_interface->set_log(log);

    simcontrol.set_incoming_interface(from_gui_interface);
    simcontrol.set_outgoing_interface(to_gui_interface);

    simcontrol.set_mission_parse(mp);
    if (!simcontrol.init()) {
        std::cout << "SimControl init() failed." << std::endl;
        return nullptr;
    }

    bool display_progress = get("display_progress", mp->params(), true);
    simcontrol.display_progress(display_progress);
    return log;
}

boost::optional<std::string> postprocess_scrimmage(
      MissionParsePtr mp, SimControl &simcontrol, std::shared_ptr<Log> &log) {

    simcontrol.output_runtime();

    // summary
    bool output_all = logging_logic(mp, "all");
    bool output_frames = logging_logic(mp, "frames");
    bool output_summary = logging_logic(mp, "summary");
    bool output_git = logging_logic(mp, "git_commits");
    bool output_mission = logging_logic(mp, "mission");
    bool output_seed = logging_logic(mp, "seed");
    bool output_nothing =
        !output_all && !output_frames && !output_summary &&
        !output_git && !output_mission && !output_seed;
    if (output_summary && !simcontrol.output_summary()) return boost::none;

    if (output_git) {
        std::map<std::string, std::unordered_set<std::string>> commits =
            simcontrol.plugin_manager()->get_commits();
        std::string scrimmage_version = get_version();

        if (scrimmage_version != "") {
            commits[scrimmage_version].insert("scrimmage");
        }

        for (auto &kv : commits) {
            std::string output = kv.first + ":";
            for (const std::string &plugin_name : kv.second) {
                output += plugin_name + ",";
            }
            output.pop_back();
            log->write_ascii(output);
        }
    }

    if (get("plot_tracks", mp->params(), false)) {
        std::string plot_cmd = "plot_3d_fr.py " + log->frames_filename();
        int result = std::system(plot_cmd.c_str());
        if (result != 0) {
            std::cout << "plot_tracks failed with return code: "
                 << result << std::endl;
        }
    }

    // Close the log file
    log->close_log();

    if (!output_nothing) {
        std::cout << "Simulation Complete" << std::endl;
    }

    simcontrol.close();
    return mp->log_dir();
}

bool check_output(std::string output_type, std::string desired_output) {
    return output_type.find("all") != std::string::npos ||
           output_type.find(desired_output) != std::string::npos;
}

std::shared_ptr<Log> setup_logging(MissionParsePtr mp) {
    std::string output_type = get("output_type", mp->params(), std::string("frames"));
    auto log = std::make_shared<Log>();
    if (check_output(output_type, "frames")) {
        log->set_enable_log(true);
        log->init(mp->log_dir(), Log::WRITE);
    } else {
        log->set_enable_log(false);
        log->init(mp->log_dir(), Log::NONE);
    }
    return log;
}

bool create_networks(const SimUtilsInfo &info, NetworkMap &networks,
                     const std::set<std::string> &plugin_tags,
                     std::function<void(std::map<std::string, std::string>&)> param_override_func) {

    for (std::string network_name : info.mp->network_names()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            info.mp->attributes()[network_name];

        PluginStatus<Network> status =
            info.plugin_manager->make_plugin<Network>(
                "scrimmage::Network", network_name, *info.file_search,
                config_parse, overrides, plugin_tags);

        if (status.status == PluginStatus<Network>::cast_failed) {
            std::cout << "Failed to load network plugin: "
                << network_name << std::endl;
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
            network->init(info.mp->params(), config_parse.params());
            networks[name] = network;
        }
    }
    return true;
}
} // namespace scrimmage
