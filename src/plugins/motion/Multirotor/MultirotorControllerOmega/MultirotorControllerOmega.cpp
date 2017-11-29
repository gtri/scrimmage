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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/plugins/motion/Multirotor/MultirotorControllerOmega/MultirotorControllerOmega.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::MultirotorControllerOmega,
                MultirotorControllerOmega_plugin)

namespace scrimmage {
namespace controller {

MultirotorControllerOmega::MultirotorControllerOmega() {
}

void MultirotorControllerOmega::init(std::map<std::string, std::string> &params) {
    u_.resize(4);
    double w = 734.85;
    // double delta = -10;
    // u_ << w+delta, w-delta, w+delta, w-delta;
    u_ << w, w, w, w;
}

bool MultirotorControllerOmega::step(double t, double dt) {
    return true;
}
} // namespace controller
} // namespace scrimmage
