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
#ifdef __clang__
_Pragma("clang diagnostic push")
_Pragma("clang diagnostic ignored \"-Wmacro-redefined\"")
_Pragma("clang diagnostic ignored \"-Wdeprecated-register\"")
#endif
#include <Python.h>
#ifdef __clang__
_Pragma("clang diagnostic pop")
#endif
#endif

#include <boost/optional.hpp>

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

    std::string mission_file = argv[optind];
    auto mp = std::make_shared<sc::MissionParse>();
    if (task_id != -1) mp->set_task_number(task_id);
    if (job_id != -1) mp->set_job_number(job_id);

    if (!mp->parse(mission_file)) {
        std::cout << "Failed to parse file: " << mission_file << std::endl;
        return -1;
    }

    if (seed_set) mp->params()["seed"] = seed;

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Initialize();
#endif

    auto log = sc::preprocess_scrimmage(mp, simcontrol);
    if (log == nullptr) {
        return -1;
    }

#if ENABLE_VIEWER == 0
    simcontrol.pause(false);
    simcontrol.run();
#else
    if (simcontrol.enable_gui()) {
        simcontrol.start();
        scrimmage::Viewer viewer;

        auto outgoing = simcontrol.outgoing_interface();
        auto incoming = simcontrol.incoming_interface();

        viewer.set_incoming_interface(outgoing);
        viewer.set_outgoing_interface(incoming);
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

    if (sc::postprocess_scrimmage(mp, simcontrol, log)) {
        return 0;
    } else {
        return -1;
    }

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Finalize();
#endif
}
