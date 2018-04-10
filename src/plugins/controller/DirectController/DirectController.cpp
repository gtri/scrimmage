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

#include <scrimmage/plugins/controller/DirectController/DirectController.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/VariableIO.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::DirectController,
                DirectController_plugin)

namespace scrimmage {
namespace controller {

void DirectController::init(std::map<std::string, std::string> &params) {
    // Discover the output variables that point to the input variables for the
    // motion model. Setup input variables that mirror the output variables.
    for (auto &kv : vars_.output_variable_index()) {
        int out_idx = vars_.declare(kv.first, VariableIO::Direction::Out);
        int in_idx = vars_.declare(kv.first, VariableIO::Direction::In);
        io_map_[out_idx] = in_idx;
    }
}

bool DirectController::step(double t, double dt) {
    // Copy over variableIO data
    for (auto &kv : io_map_) {
        vars_.output(kv.first, vars_.input(kv.second));
    }
    return true;
}
} // namespace controller
} // namespace scrimmage
