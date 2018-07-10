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
#include <scrimmage/plugins/controller/JSBSimControlControllerHeadingPID/JSBSimControlControllerHeadingPID.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/State.h>

#include <iostream>

#include <boost/algorithm/string.hpp>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::JSBSimControlControllerHeadingPID, JSBSimControlControllerHeadingPID_plugin)

namespace scrimmage {
namespace controller {

namespace sc = scrimmage;
using ang = scrimmage::Angles;

void JSBSimControlControllerHeadingPID::init(std::map<std::string, std::string> &params) {
    angles_from_jsbsim_.set_input_clock_direction(ang::Rotate::CW);
    angles_from_jsbsim_.set_input_zero_axis(ang::HeadingZero::Pos_Y);
    angles_from_jsbsim_.set_output_clock_direction(ang::Rotate::CCW);
    angles_from_jsbsim_.set_output_zero_axis(ang::HeadingZero::Pos_X);

    angles_to_jsbsim_.set_input_clock_direction(ang::Rotate::CCW);
    angles_to_jsbsim_.set_input_zero_axis(ang::HeadingZero::Pos_X);
    angles_to_jsbsim_.set_output_clock_direction(ang::Rotate::CW);
    angles_to_jsbsim_.set_output_zero_axis(ang::HeadingZero::Pos_Y);

    if (!roll_pid_.init(params["roll_pid"], true)) {
        std::cout << "Failed to parse roll_pid." << std::endl;
    }
    if (!pitch_pid_.init(params["pitch_pid"], true)) {
        std::cout << "Failed to parse pitch_pid." << std::endl;
    }
    if (!yaw_pid_.init(params["yaw_pid"], true)) {
        std::cout << "Failed to parse yaw_pid." << std::endl;
    }

    // Setup variable index for controllers
    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::Out);
    aileron_idx_ = vars_.declare(VariableIO::Type::aileron, VariableIO::Direction::Out);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::Out);
}

bool JSBSimControlControllerHeadingPID::step(double t, double dt) {

    // Roll stabilizer
    roll_pid_.set_setpoint(0);
    double u_roll = roll_pid_.step(dt, state_->quat().roll());
    vars_.output(aileron_idx_, ang::angle_pi(u_roll));

    // Pitch stabilizer
    pitch_pid_.set_setpoint(0);
    vars_.output(elevator_idx_, -pitch_pid_.step(dt, state_->quat().pitch()));

    // Yaw stabilizer
    double desired_yaw = desired_state_->quat().yaw();
    angles_to_jsbsim_.set_angle(ang::rad2deg(desired_yaw));
    yaw_pid_.set_setpoint(ang::deg2rad(angles_to_jsbsim_.angle()));
    vars_.output(rudder_idx_, -yaw_pid_.step(dt, state_->quat().yaw()));

    return true;
}
} // namespace controller
} // namespace scrimmage
