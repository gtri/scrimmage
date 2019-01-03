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

#include <pybind11/pybind11.h>
using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace py = pybind11;

void importer(const std::string &module) {
    py::print("importing " + module);
    py::module::import(module.c_str());
    py::print("done importing " + module);
}


int main(int argc, char *argv[]) {
    Py_Initialize();
    importer("numpy");
    importer("tensorflow");
    Py_Finalize();
    sc::SimControl simcontrol;
}
