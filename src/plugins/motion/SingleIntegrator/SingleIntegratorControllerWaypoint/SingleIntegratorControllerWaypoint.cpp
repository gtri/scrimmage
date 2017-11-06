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
#include <scrimmage/plugins/motion/SingleIntegrator/SingleIntegratorControllerWaypoint/SingleIntegratorControllerWaypoint.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::SingleIntegratorControllerWaypoint, SingleIntegratorControllerWaypoint_plugins)

namespace scrimmage {
namespace controller {

namespace sc = scrimmage;

void SingleIntegratorControllerWaypoint::init(std::map<std::string, std::string> &params) {
    gain_ = sc::get("gain", params, 1.0);
}

bool SingleIntegratorControllerWaypoint::step(double t, double dt) {
    u_ = gain_ * (desired_state_->pos() - state_->pos());
    return true;
}

} // namespace controller
} // namespace scrimmage
