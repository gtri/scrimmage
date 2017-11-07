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
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/motion/DoubleIntegrator/DoubleIntegratorControllerVelYaw/DoubleIntegratorControllerVelYaw.h>

#include <iostream>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::DoubleIntegratorControllerVelYaw, DoubleIntegratorControllerVelYaw_plugin)

namespace scrimmage {
namespace controller {

void set_pid(sc::PID &pid, std::string str, bool is_angle) {
    std::vector<std::string> str_vals;
    boost::split(str_vals, str, boost::is_any_of(","));

    if (str_vals.size() != 4) {
        std::cout << "error parsing in SimpleAircraftControllerPID" << std::endl;
    } else {
        double p = std::stod(str_vals[0]);
        double i = std::stod(str_vals[1]);
        double d = std::stod(str_vals[2]);
        pid.set_parameters(p, i, d);

        if (is_angle) {
            double i_lim = sc::Angles::deg2rad(std::stod(str_vals[3]));
            pid.set_integral_band(i_lim);
            pid.set_is_angle(true);
        } else {
            double i_lim = std::stod(str_vals[3]);
            pid.set_integral_band(i_lim);
        }
    }
}

void DoubleIntegratorControllerVelYaw::init(std::map<std::string, std::string> &params) {
    set_pid(yaw_pid_, params["yaw_pid"], true);

    for (int i = 0; i < 3; i++) {
        vel_pids_.push_back(sc::PID());
        set_pid(vel_pids_[i], params["vel_pid"], false);
    }
}

bool DoubleIntegratorControllerVelYaw::step(double t, double dt) {
    for (int i = 0; i < 3; i++) {
        vel_pids_[i].set_setpoint(desired_state_->vel()(i));
        u_(i) = vel_pids_[i].step(dt, state_->vel()(i));
    }

    yaw_pid_.set_setpoint(desired_state_->quat().yaw());
    u_(3) = yaw_pid_.step(dt, state_->quat().yaw());

    return true;
}
} // namespace controller
} // namespace scrimmage
