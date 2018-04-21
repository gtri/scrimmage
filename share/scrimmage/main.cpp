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
#if ENABLE_VIEWER == 1
#include <scrimmage/viewer/Viewer.h>
#endif

#include <scrimmage/log/Log.h>

#if ENABLE_PYTHON_BINDINGS == 1
#include <Python.h>
#endif

using std::cout;
using std::endl;

namespace sc = scrimmage;

sc::SimControl simcontrol;

// Handle kill signal
void HandleSignal(int s) {
    cout << endl << "Exiting gracefully" << endl;
    simcontrol.force_exit();
}

int main(int argc, char *argv[]) {
    // Handle kill signals
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = HandleSignal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int job_id = -1;
    int task_id = -1;

    bool seed_set = false;
    std::string seed = "";

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

    // Parse mission config file:
    sc::MissionParsePtr mp = std::make_shared<sc::MissionParse>();

    // If the task_num was defined, tell the mission parser, so it can be
    // stored in the directory log name
    if (task_id != -1) {
        mp->set_task_number(task_id);
    }
    if (job_id != -1) {
        mp->set_job_number(job_id);
    }
    if (!mp->parse(argv[optind])) {
        cout << "Failed to parse file: " << argv[optind] << endl;
        return -1;
    }

    std::string output_type = sc::get("output_type", mp->params(), std::string("frames"));
    bool output_all = output_type.find("all") != std::string::npos;
    auto should_log = [&](std::string s) {
        return output_all || output_type.find(s) != std::string::npos;
    };

    bool output_frames = should_log("frames");
    bool output_summary = should_log("summary");
    bool output_git = should_log("git_commits");
    bool output_mission = should_log("mission");
    bool output_seed = should_log("seed");
    bool output_nothing =
        !output_all && !output_frames && !output_summary &&
        !output_git && !output_mission && !output_seed;

    if (!output_nothing) {
        mp->create_log_dir();
    }

    auto log = sc::setup_logging(mp);

    // Overwrite the seed if it's set
    if (seed_set) {
        mp->params()["seed"] = seed;
    }
    simcontrol.set_log(log);

    sc::InterfacePtr to_gui_interface = std::make_shared<sc::Interface>();
    sc::InterfacePtr from_gui_interface = std::make_shared<sc::Interface>();
    to_gui_interface->set_log(log);
    from_gui_interface->set_log(log);

    simcontrol.set_incoming_interface(from_gui_interface);
    simcontrol.set_outgoing_interface(to_gui_interface);

    // Split off SimControl in it's own thread
#if ENABLE_PYTHON_BINDINGS == 1
    Py_Initialize();
#endif
    simcontrol.set_mission_parse(mp);
    if (!simcontrol.init()) {
        cout << "SimControl init() failed." << endl;
        return -1;
    }

    bool display_progress = sc::get("display_progress", mp->params(), true);
    simcontrol.display_progress(display_progress);

#if ENABLE_VIEWER == 0
    simcontrol.pause(false);
    simcontrol.run();
#else
    if (simcontrol.enable_gui()) {
        simcontrol.start();
        scrimmage::Viewer viewer;
        viewer.set_incoming_interface(to_gui_interface);
        viewer.set_outgoing_interface(from_gui_interface);
        viewer.set_enable_network(false);
        viewer.init(mp->attributes()["camera"], mp->log_dir(), mp->dt());
        viewer.run();

        // When the viewer finishes, tell simcontrol to exit
        simcontrol.force_exit();
        simcontrol.join();
    } else {
        simcontrol.pause(false);
        simcontrol.run();
    }
#endif

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Finalize();
#endif
    simcontrol.output_runtime();

    // summary
    if (output_summary && !simcontrol.output_summary()) return -1;

    if (output_git) {
        std::map<std::string, std::unordered_set<std::string>> commits =
            simcontrol.plugin_manager()->get_commits();
        std::string scrimmage_version = sc::get_version();

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

    if (sc::get("plot_tracks", mp->params(), false)) {
        std::string plot_cmd = "plot_3d_fr.py " + log->frames_filename();
        int result = std::system(plot_cmd.c_str());
        if (result != 0) {
            cout << "plot_tracks failed with return code: "
                 << result << endl;
        }
    }

    // Close the log file
    log->close_log();

    cout << "Simulation Complete" << endl;
    return 0;
}
