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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_set>

#include <signal.h>

#include "scrimmage/autonomy/Autonomy.h"
#include "scrimmage/common/Utilities.h"
#include "scrimmage/entity/Contact.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/metrics/Metrics.h"
#include "scrimmage/network/Interface.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/PluginManager.h"
#include "scrimmage/simcontrol/SimControl.h"
#include "scrimmage/simcontrol/SimUtils.h"
#if ENABLE_VTK == 1
#include "scrimmage/viewer/Viewer.h"
#endif

#if ENABLE_UNITY_BRIDGE == 1
#include "scrimmage/unity_bridge/UnityBridge.h"
#endif

#include <boost/optional.hpp>

#include "scrimmage/log/Log.h"

using std::cout;
using std::endl;

namespace sc = scrimmage;

// Callback function for shutdown
namespace {

// https://stackoverflow.com/a/48164204
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) {
    shutdown_handler(signal);
}

}  // namespace

int main(int argc, char* argv[]) {
    sc::SimControl simcontrol;

    // Handle kill signals
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    shutdown_handler = [&](int /*s*/) {
        cout << endl << "Exiting gracefully" << endl;
        simcontrol.force_exit();
    };
    sa.sa_handler = signal_handler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int job_id = -1;
    int task_id = -1;

    bool seed_set = false;
    std::string seed = "";
    std::ostringstream overrides;
    bool first_override = true;

    int opt;
    while ((opt = getopt(argc, argv, "t:j:s:")) != -1) {
        switch (opt) {
            case 't':
                task_id = std::stoi(std::string(optarg));
                break;
            case 'j':
                job_id = std::stoi(std::string(optarg));
                break;
            case 's':
                seed = std::string(optarg);
                seed_set = true;
                break;
            case '?':
                if (optopt == 't') {
                    fprintf(stderr, "Option -%d requires an integer argument.\n", optopt);
                } else {
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                }
                return 1;
            default:
                exit(EXIT_FAILURE);
        }
    }

    if (optind >= argc || argc < 2) {
        cout << "usage: " << argv[0] << " [var:=value ...] scenario.xml [var:=value ...]" << endl;
        return -1;
    }

    // Parse positional arguments: args with ":=" are overrides, first without is mission file
    std::string mission_file;
    for (int i = optind; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find(":=") != std::string::npos) {
            // This is an override - normalize := to =
            if (!first_override) {
                overrides << ",";
            }
            first_override = false;
            // Replace := with =
            std::string normalized = arg;
            size_t pos = normalized.find(":=");
            normalized.replace(pos, 2, "=");
            overrides << normalized;
        } else if (mission_file.empty()) {
            // First non-override arg is the mission file
            mission_file = arg;
        }
    }

    if (mission_file.empty()) {
        std::cerr << "Error: No mission file specified" << endl;
        std::cerr << "usage: " << argv[0] << " [var:=value ...] scenario.xml [var:=value ...]" << endl;
        return -1;
    }

    // Overwrite mission parameters from command line
    if (task_id != -1)
        simcontrol.mp()->set_task_number(task_id);
    if (job_id != -1)
        simcontrol.mp()->set_job_number(job_id);
    simcontrol.mp()->set_overrides(overrides.str());
    if (not simcontrol.init(mission_file)) {
        cout << "Failed to initialize SimControl with mission file: " << mission_file << endl;
        return -1;
    }

    if (seed_set)
        simcontrol.mp()->params()["seed"] = seed;

    simcontrol.run_send_shapes();  // draw any intial shapes

#if ENABLE_UNITY_BRIDGE == 1
    {
        auto it = simcontrol.mp()->attributes().find("unity_bridge");
        bool unity_enabled = it != simcontrol.mp()->attributes().end()
                             && sc::get<bool>("enabled", it->second, false);

        if (unity_enabled) {
            const auto& up = it->second;
            auto unity_bridge = std::make_shared<scrimmage::unity_bridge::UnityBridge>();

            unity_bridge->set_pub_address(
                "tcp://*:" + std::to_string(sc::get<int>("pub_port", up, 10250)));
            unity_bridge->set_sub_address(
                "tcp://*:" + std::to_string(sc::get<int>("sub_port", up, 10251)));
            unity_bridge->set_connection_timeout_s(
                sc::get<double>("connection_timeout_s", up, 10.0));
            unity_bridge->set_sim_dt(simcontrol.mp()->dt());
            unity_bridge->set_origin(
                simcontrol.mp()->latitude_origin(),
                simcontrol.mp()->longitude_origin(),
                simcontrol.mp()->altitude_origin());

            if (!unity_bridge->bind()) {
                std::cerr << "WARNING: UnityBridge bind failed; "
                             "continuing without Unity." << std::endl;
            } else if (!unity_bridge->send_handshake(0.0)) {
                std::cerr << "WARNING: Unity handshake failed; "
                             "continuing without Unity." << std::endl;
            } else {
                // Helper: convert Contact::Type to ICD contact_type string
                auto contact_type_str = [](sc::Contact::Type t) -> std::string {
                    switch (t) {
                        case sc::Contact::Type::AIRCRAFT:  return "aircraft";
                        case sc::Contact::Type::QUADROTOR: return "quadrotor";
                        case sc::Contact::Type::SPHERE:    return "sphere";
                        case sc::Contact::Type::MESH:      return "mesh";
                        default:                           return "unknown";
                    }
                };

                // Spawn initial entities
                for (auto& entity : simcontrol.ents()) {
                    int entity_id  = entity->id().id();
                    auto& ent_id_map = simcontrol.mp()->ent_id_to_block_id();
                    auto desc_it = ent_id_map.find(entity_id);
                    if (desc_it == ent_id_map.end()) continue;
                    int desc_id = desc_it->second;

                    std::string prefab_override;
                    auto& attrs = simcontrol.mp()->entity_attributes();
                    auto attrs_it = attrs.find(desc_id);
                    if (attrs_it != attrs.end()) {
                        auto uv_it = attrs_it->second.find("unity_visual");
                        if (uv_it != attrs_it->second.end()) {
                            auto pid_it = uv_it->second.find("prefab_id");
                            if (pid_it != uv_it->second.end()) {
                                prefab_override = pid_it->second;
                            }
                        }
                    }

                    auto cfg = scrimmage::unity_bridge::entity_config_from_contact_visual(
                        *entity->contact_visual(),
                        entity->id().team_id(),
                        entity->id().sub_swarm_id(),
                        contact_type_str(entity->type()),
                        prefab_override);

                    unity_bridge->send_entity_create(0.0, cfg);
                }

                simcontrol.set_unity_bridge(unity_bridge);
            }
        }
    }
#endif

    std::shared_ptr<std::thread> viewer_thread = nullptr;

#if ENABLE_VTK == 0
    // If the GUI wasn't built, un-pause by default.
    simcontrol.pause(false);
#else
    // If the GUI is enabled, the viewer will be run in a separate thread. Use
    // a shared_ptr to keep it "in scope", if it is created.
    std::shared_ptr<scrimmage::Viewer> viewer = nullptr;

    if (simcontrol.enable_gui()) {
        viewer = std::make_shared<scrimmage::Viewer>();

        auto outgoing = simcontrol.outgoing_interface();
        auto incoming = simcontrol.incoming_interface();

        viewer->set_incoming_interface(outgoing);
        viewer->set_outgoing_interface(incoming);
        viewer->set_enable_network(false);

        // Get the camera params from mission file, if they exist
        std::map<std::string, std::string> camera_params;
        auto it_camera = simcontrol.mp()->attributes().find("camera");
        if (it_camera != simcontrol.mp()->attributes().end()) {
            camera_params = it_camera->second;
        }

        // Initialize the VTK GUI viewer
        if (!viewer->init(simcontrol.mp(), camera_params)) {
            return -1;
        }

        // Run the viewer in its own thread
        auto viewer_thread_func = [&]() { viewer->run(); };
        viewer_thread = std::make_shared<std::thread>(viewer_thread_func);

    } else {
        // If the GUI isn't enabled, un-pause by default.
        simcontrol.pause(false);
    }
#endif

    // Run SimControl::run() blocking function, which steps through simulation
    if (not simcontrol.run()) {
        cout << "SimControl::run() failed." << endl;
        return -1;
    }

    if (not simcontrol.shutdown()) {
        cout << "Failed to shutdown properly." << endl;
        return -1;
    }

    // Join the viewer thread, if it was created
    if (viewer_thread != nullptr) {
        viewer_thread->join();
    }

    return 0;
}
