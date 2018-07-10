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

#include <scrimmage/plugins/controller/FixedWing6DOFControllerPID/FixedWing6DOFControllerPID.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Angles.h>
#include <vector>
#include <iostream>
#include <boost/algorithm/string.hpp>

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::Controller,
        scrimmage::controller::FixedWing6DOFControllerPID,
        FixedWing6DOFControllerPID_plugin)

namespace scrimmage {
namespace controller {

namespace sc = scrimmage;

void set_pid(sc::PID &pid, std::string str, bool is_angle) {
    std::vector<std::string> str_vals;
    boost::split(str_vals, str, boost::is_any_of(","));

    if (str_vals.size() != 4) {
        std::cout << "error parsing in FixedWing6DOFControllerPID" << std::endl;
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

void FixedWing6DOFControllerPID::init(
        std::map<std::string, std::string> &params) {
    set_pid(heading_pid_, params["heading_pid"], true);
    set_pid(alt_pid_, params["alt_pid"], false);
    set_pid(vel_pid_, params["vel_pid"], false);

    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::Out);
    aileron_idx_ = vars_.declare(VariableIO::Type::aileron, VariableIO::Direction::Out);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::Out);
}

bool FixedWing6DOFControllerPID::step(double t, double dt) {
    // double desired_yaw = desired_state_->quat().yaw();
    //
    // heading_pid_.set_setpoint(desired_yaw);
    // double u_heading = heading_pid_.step(dt, state_->quat().yaw());
    // double roll_error = u_heading + state_->quat().roll();
    //
    // alt_pid_.set_setpoint(desired_state_->pos()(2));
    // double u_alt = alt_pid_.step(dt, state_->pos()(2));
    // double pitch_error = (-u_alt - state_->quat().pitch());
    //
    // vel_pid_.set_setpoint(desired_state_->vel()(0));
    // double u_throttle = vel_pid_.step(dt, state_->vel().norm());

    // u_ << u_throttle, roll_error, pitch_error, 0;
    // throttle, elevator, aileron, rudder

#if 1
    // Rudder dublet (TODO)
    if (t >= 2 && t <= 2.4) {
        vars_.output(rudder_idx_, -0.26);
    } else if (t >= 2.4 && t <= 2.8) {
        vars_.output(rudder_idx_, +0.26);
    } else {
        vars_.output(rudder_idx_, 0);
    }
#endif

#if 1
    // Aileron dublet
    if (t >= 8 && t <= 8.4) {
        vars_.output(aileron_idx_, -0.5);
    } else if (t >= 8.4 && t <= 8.8) {
        vars_.output(aileron_idx_, 0.5);
    } else {
        vars_.output(aileron_idx_, 0);
    }
#endif

    return true;
}
}  // namespace controller
}  // namespace scrimmage
