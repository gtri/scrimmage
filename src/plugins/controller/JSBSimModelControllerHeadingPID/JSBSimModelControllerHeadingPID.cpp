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

#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/JSBSimModelControllerHeadingPID/JSBSimModelControllerHeadingPID.h>

#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::JSBSimModelControllerHeadingPID, JSBSimModelControllerHeadingPID_plugin)

namespace scrimmage {
namespace controller {

using ang = scrimmage::Angles;

void JSBSimModelControllerHeadingPID::init(std::map<std::string, std::string> &params) {
    angles_from_jsbsim_.set_input_clock_direction(ang::Rotate::CW);
    angles_from_jsbsim_.set_input_zero_axis(ang::HeadingZero::Pos_Y);
    angles_from_jsbsim_.set_output_clock_direction(ang::Rotate::CCW);
    angles_from_jsbsim_.set_output_zero_axis(ang::HeadingZero::Pos_X);

    angles_to_jsbsim_.set_input_clock_direction(ang::Rotate::CCW);
    angles_to_jsbsim_.set_input_zero_axis(ang::HeadingZero::Pos_X);
    angles_to_jsbsim_.set_output_clock_direction(ang::Rotate::CW);
    angles_to_jsbsim_.set_output_zero_axis(ang::HeadingZero::Pos_Y);

    std::string pid_gains_ = scrimmage::get("heading_pid", params, "1.0, 0.5, 0.0, 4.5");
    heading_pid_.init(pid_gains_, true);

    heading_lag_initialized_ = false;

    max_bank_ = ang::deg2rad(std::stod(params.at("max_bank")));

    input_vel_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    input_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);
    input_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::In);

    output_vel_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    output_bank_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::Out);

    // Is the motion model using pitch or altitude control
    std::string z_name = vars_.exists(VariableIO::Type::desired_pitch, VariableIO::Direction::Out) ?
        vars_.type_map().at(VariableIO::Type::desired_pitch) :
        vars_.type_map().at(VariableIO::Type::desired_altitude);

    output_alt_idx_ = vars_.declare(z_name, VariableIO::Direction::Out);
}

bool JSBSimModelControllerHeadingPID::step(double t, double dt) {
    double desired_yaw = vars_.input(input_heading_idx_);
    angles_to_jsbsim_.set_angle(ang::rad2deg(desired_yaw));
    desired_yaw = ang::deg2rad(angles_to_jsbsim_.angle());
    desired_yaw = ang::angle_pi(desired_yaw);

    double yaw = state_->quat().yaw();
    angles_to_jsbsim_.set_angle(ang::rad2deg(yaw));
    yaw = ang::deg2rad(angles_to_jsbsim_.angle());
    yaw = ang::angle_pi(yaw);

    // TODO: Chatter occurs when a command heading changes by 180 degrees
    // Might have to use the lag
    double desired_yaw_lag;
    if (heading_lag_initialized_) {
        double k = 1.00; // disable lag
        if (ang::angle_pi(std::abs(desired_yaw - yaw)) > 2.35619) {
            k = dt * 0.75;
        }
        desired_yaw_lag = desired_yaw * k + (1.0 - k) * prev_desired_yaw_;
    } else {
        desired_yaw_lag = desired_yaw;
        if (!std::isnan(desired_yaw_lag)) {
            heading_lag_initialized_ = true;
        }
    }
    prev_desired_yaw_ = desired_yaw_lag;

    // Convert desired heading to bank angle:

    heading_pid_.set_setpoint(desired_yaw_lag);
    double bank_cmd = heading_pid_.step(dt, yaw);

    bank_cmd = boost::algorithm::clamp(bank_cmd, -max_bank_, max_bank_);

    // save what was used as the input
    vars_.output(output_vel_idx_, vars_.input(input_vel_idx_));
    vars_.output(output_bank_idx_, bank_cmd);
    vars_.output(output_alt_idx_, vars_.input(input_alt_idx_));

    return true;
}

} // namespace controller
} // namespace scrimmage
