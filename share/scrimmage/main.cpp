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
#include <cstring>
#include <functional>
#include <getopt.h>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <signal.h>

#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/simcontrol/SimControl.h"
#include "scrimmage/viewer/VisualizationSession.h"

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

    bool use_ogre = false;

    static struct option long_options[] = {
        {"ogre", no_argument, 0, 1000},
        {0, 0, 0, 0},
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "t:j:s:", long_options, nullptr)) != -1) {
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
            case 1000:
                use_ogre = true;
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
           cout << "usage: " << argv[0]
               << " [--ogre] [var:=value ...] scenario.xml [var:=value ...]" << endl;
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
        std::cerr << "usage: " << argv[0]
                  << " [--ogre] [var:=value ...] scenario.xml [var:=value ...]" << endl;
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

    std::shared_ptr<std::thread> viewer_thread = nullptr;

    std::unique_ptr<scrimmage::VisualizationSession> viewer = nullptr;

    if (simcontrol.enable_gui()) {
        auto backend = use_ogre ? scrimmage::ViewerBackendType::OGRE_NEXT
                                : scrimmage::ViewerBackendType::VTK;

        if (!scrimmage::VisualizationSession::backend_available(backend)) {
            std::cerr << "Error: requested GUI backend '"
                      << scrimmage::VisualizationSession::backend_name(backend)
                      << "' is not available in this build." << std::endl;
            return -1;
        }

        viewer = std::make_unique<scrimmage::VisualizationSession>(backend);

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

        if (!viewer->init(simcontrol.mp(), camera_params)) {
            return -1;
        }

        auto viewer_thread_func = [&]() { viewer->run(); };
        viewer_thread = std::make_shared<std::thread>(viewer_thread_func);

    } else {
        // If the GUI isn't enabled, un-pause by default.
        simcontrol.pause(false);
    }

    // Run SimControl::run() blocking function, which steps through simulation
    auto stop_and_join_viewer = [&]() {
        if (viewer != nullptr) {
            viewer->stop();
        }
        if (viewer_thread != nullptr && viewer_thread->joinable()) {
            viewer_thread->join();
        }
    };

    if (not simcontrol.run()) {
        stop_and_join_viewer();
        cout << "SimControl::run() failed." << endl;
        return -1;
    }

    stop_and_join_viewer();

    if (not simcontrol.shutdown()) {
        cout << "Failed to shutdown properly." << endl;
        return -1;
    }

    return 0;
}
