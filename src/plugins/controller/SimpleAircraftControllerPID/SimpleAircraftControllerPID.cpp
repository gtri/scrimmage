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
#include <scrimmage/plugins/controller/SimpleAircraftControllerPID/SimpleAircraftControllerPID.h>

#include <iostream>

#include <boost/algorithm/string.hpp>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::SimpleAircraftControllerPID, SimpleAircraftControllerPID_plugin)

namespace scrimmage {
namespace controller {

namespace sc = scrimmage;

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

void SimpleAircraftControllerPID::init(std::map<std::string, std::string> &params) {
    set_pid(heading_pid_, params["heading_pid"], true);
    set_pid(alt_pid_, params["alt_pid"], false);
    set_pid(vel_pid_, params["vel_pid"], false);
    use_roll_ = sc::str2bool(params.at("use_roll"));

    std::string ctrl_name = use_roll_ ?
        vars_.type_map().at(VariableIO::Type::desired_roll) :
        vars_.type_map().at(VariableIO::Type::desired_heading);

    input_roll_or_heading_idx_ = vars_.declare(ctrl_name, VariableIO::Direction::In);
    input_altitude_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::In);
    input_velocity_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);

    output_throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);
    output_roll_rate_idx_ = vars_.declare(VariableIO::Type::roll_rate, VariableIO::Direction::Out);
    output_pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);
}

bool SimpleAircraftControllerPID::step(double t, double dt) {
    double roll_error;
    if (use_roll_) {
        heading_pid_.set_setpoint(vars_.input(input_roll_or_heading_idx_));
        roll_error = -heading_pid_.step(dt, state_->quat().roll());
    } else {
        double desired_yaw = vars_.input(input_roll_or_heading_idx_);

        heading_pid_.set_setpoint(desired_yaw);
        double u_heading = heading_pid_.step(dt, state_->quat().yaw());
        roll_error = u_heading + state_->quat().roll();
    }

    alt_pid_.set_setpoint(vars_.input(input_altitude_idx_));
    double u_alt = alt_pid_.step(dt, state_->pos()(2));
    double pitch_error = (-u_alt - state_->quat().pitch());

    vel_pid_.set_setpoint(vars_.input(input_velocity_idx_));
    double u_throttle = vel_pid_.step(dt, state_->vel().norm());

    vars_.output(output_throttle_idx_, u_throttle);
    vars_.output(output_roll_rate_idx_, roll_error);
    vars_.output(output_pitch_rate_idx_, pitch_error);
    return true;
}
} // namespace controller
} // namespace scrimmage
