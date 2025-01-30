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

#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/SimUtils.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/metrics/Metrics.h>

#include <signal.h>
#include <cstdlib>

#include <iostream>
#include <ctime>

#include <unordered_set>
#include <string>
#include <ostream>
#include <memory>
#if ENABLE_VTK == 1
#include <scrimmage/viewer/Viewer.h>
#endif

#include <scrimmage/log/Log.h>

#include <boost/optional.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

// Callback function for shutdown
namespace {
// https://stackoverflow.com/a/48164204
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
} // namespace

int main(int argc, char *argv[]) {
    sc::SimControl simcontrol;

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

    int job_id = -1;
    int task_id = -1;

    bool seed_set = false;
    std::string seed = "";
    std::string overrides = "";

    int opt;
    while ((opt = getopt(argc, argv, "t:j:s:o:")) != -1) {
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
        case 'o':
            overrides = std::string(optarg);
            break;
        case '?':
            if (optopt == 't') {
                fprintf(stderr, "Option -%d requires an integer argument.\n", optopt);
            } else {
                fprintf(stderr,
                         "Unknown option character `\\x%x'.\n",
                         optopt);
            }
            return 1;
        default:
            exit(EXIT_FAILURE);
        }
    }

    if (optind >= argc || argc < 2) {
        cout << "usage: " << argv[0] << " scenario.xml" << endl;
        return -1;
    }

    // Overwrite mission parameters from command line
    if (task_id != -1) simcontrol.mp()->set_task_number(task_id);
    if (job_id != -1) simcontrol.mp()->set_job_number(job_id);
    simcontrol.mp()->set_overrides(overrides);

    // Load in the mission file and parse mission parameters
    std::string mission_file = argv[optind];
    if (not simcontrol.init(mission_file)) {
        cout << "Failed to initialize SimControl with mission file: "
             << mission_file << endl;
        return -1;
    }

    if (seed_set) simcontrol.mp()->params()["seed"] = seed;

    simcontrol.run_send_shapes(); // draw any intial shapes

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
        viewer->init(simcontrol.mp(), camera_params);

        // Run the viewer in its own thread
        auto viewer_thread_func = [&] () {
            viewer->run();
        };
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
