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
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/motion/DoubleIntegrator/DoubleIntegratorControllerWaypoint/DoubleIntegratorControllerWaypoint.h>

#include <iostream>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::DoubleIntegratorControllerWaypoint, DoubleIntegratorControllerWaypoint_plugin)

namespace scrimmage {
namespace controller {

void DoubleIntegratorControllerWaypoint::init(std::map<std::string, std::string> &params) {
    std::vector<double> gain;
    if (!scrimmage::str2vec<double>(params.at("gain"), ",", gain, 2)) {
        std::cout << "warning: did not get gain properly in DoubleIntegratorControllerWaypoint" << std::endl;
    } else {
        gain_ << gain[0], gain[1];
    }
}

bool DoubleIntegratorControllerWaypoint::step(double t, double dt) {
    u_.head(3) = -gain_(0) * (state_->pos() - desired_state_->pos()) - gain_(1) * state_->vel();
    return true;
}
} // namespace controller
} // namespace scrimmage
