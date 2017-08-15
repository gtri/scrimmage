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

#include <scrimmage/math/Angles.h>

#include "WayPoint.h"

REGISTER_PLUGIN(scrimmage::Autonomy, WayPoint, WayPoint_plugin)

WayPoint::WayPoint() {}

void WayPoint::init(std::map<std::string, std::string> &params) {
    desired_state_->vel() = 21*Eigen::Vector3d::UnitX();
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = state_->pos()(2)*Eigen::Vector3d::UnitZ();
}

bool WayPoint::step_autonomy(double t, double dt) {
    if (t < 4) {
    } else if (t < 20) {
        desired_state_->quat().set(0, 0, scrimmage::Angles::deg2rad(90));
    } else if (t < 50) {
    }

    return true;
}
