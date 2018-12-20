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

#include <scrimmage/log/Log.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/SimUtils.h>

#if ENABLE_VIEWER == 1
#include <scrimmage/viewer/Viewer.h>
#endif

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

#include <string>

#include <boost/optional.hpp>

boost::optional<std::string> scrimmage_main_run(scrimmage::SimControl &simcontrol,
         const std::string &mission_file,
         int job_id,
         int task_id,
         const std::string &seed) {

    auto mp = std::make_shared<scrimmage::MissionParse>();
    if (task_id != -1) mp->set_task_number(task_id);
    if (job_id != -1) mp->set_job_number(job_id);

    if (!mp->parse(mission_file)) {
        std::cout << "Failed to parse file: " << mission_file << std::endl;
        return boost::none;
    }

    if (seed != "") mp->params()["seed"] = seed;

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Initialize();
#endif

    auto log = preprocess_scrimmage(mp, simcontrol);
    simcontrol.send_terrain();
    simcontrol.run_send_shapes(); // draw any intial shapes

    if (log == nullptr) {
        return boost::none;
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
    return postprocess_scrimmage(mp, simcontrol, log);
}
