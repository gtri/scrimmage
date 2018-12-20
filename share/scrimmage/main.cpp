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

#include "scrimmage_main.h"

#include <iostream>

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

    auto log_dir = scrimmage_main_run(simcontrol, mission_file, job_id, task_id, seed);

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Finalize();
#endif

    return log_dir ? 0 : -1;
}
