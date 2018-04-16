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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/Unicycle3DControllerPID/Unicycle3DControllerPID.h>

#include <iostream>

#include <boost/algorithm/string.hpp>

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::Unicycle3DControllerPID,
                Unicycle3DControllerPID_plugin)

namespace scrimmage {
namespace controller {

void set_pid(PID &pid, std::string str, bool is_angle) {
    std::vector<std::string> str_vals;
    boost::split(str_vals, str, boost::is_any_of(","));

    if (str_vals.size() != 4) {
        std::cout << "error parsing in Unicycle3DControllerPID" << std::endl;
    } else {
        double p = std::stod(str_vals[0]);
        double i = std::stod(str_vals[1]);
        double d = std::stod(str_vals[2]);
        pid.set_parameters(p, i, d);

        if (is_angle) {
            double i_lim = Angles::deg2rad(std::stod(str_vals[3]));
            pid.set_integral_band(i_lim);
            pid.set_is_angle(true);
        } else {
            double i_lim = std::stod(str_vals[3]);
            pid.set_integral_band(i_lim);
        }
    }
}

void Unicycle3DControllerPID::init(std::map<std::string, std::string> &params) {
    set_pid(heading_pid_, params["heading_pid"], true);
    set_pid(alt_pid_, params["alt_pid"], false);
    set_pid(vel_pid_, params["vel_pid"], false);
    use_roll_ = str2bool(params.at("use_roll"));
    u_ = std::make_shared<Eigen::Vector3d>();

    desired_alt_idx_ = vars_.declare("desired_altitude", VariableIO::Direction::In);
    desired_speed_idx_ = vars_.declare("desired_speed", VariableIO::Direction::In);
    desired_heading_idx_ = vars_.declare("desired_heading", VariableIO::Direction::In);

    if (vars_.exists("acceleration", VariableIO::Direction::Out)) {
        accel_idx_ = vars_.declare("acceleration", VariableIO::Direction::Out);
        velocity_idx_ = -1;
    } else {
        velocity_idx_ = vars_.declare("velocity", VariableIO::Direction::Out);
        accel_idx_ = -1;
    }
    turn_rate_idx_ = vars_.declare("turn_rate", VariableIO::Direction::Out);
    pitch_rate_idx_ = vars_.declare("pitch_rate", VariableIO::Direction::Out);
}

bool Unicycle3DControllerPID::step(double t, double dt) {
    heading_pid_.set_setpoint(vars_.input(desired_heading_idx_));
    double u_heading = heading_pid_.step(dt, state_->quat().yaw());

    alt_pid_.set_setpoint(vars_.input(desired_alt_idx_));
    double u_alt = alt_pid_.step(dt, state_->pos()(2));
    double pitch_error = (-u_alt - state_->quat().pitch());

    // If the motion model allows us to set speed directly, set it directly. If
    // not, use a PID controller to set the acceleration.
    double desired_speed = vars_.input(desired_speed_idx_);
    if (accel_idx_ < 0) {
        // Directly set velocity
        vars_.output(velocity_idx_, desired_speed);
    } else {
        // Use a PID controller to control velocity
        vel_pid_.set_setpoint(desired_speed);
        double u_thrust = vel_pid_.step(dt, state_->vel().norm());
        vars_.output(accel_idx_, u_thrust);
    }
    vars_.output(turn_rate_idx_, u_heading);
    vars_.output(pitch_rate_idx_, pitch_error);

    return true;
}
} // namespace controller
} // namespace scrimmage
