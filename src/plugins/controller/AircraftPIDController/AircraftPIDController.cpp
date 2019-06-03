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

#include <scrimmage/plugins/controller/AircraftPIDController/AircraftPIDController.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/clamp.hpp>

using std::cout;
using std::endl;
using boost::algorithm::clamp;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::AircraftPIDController,
                AircraftPIDController_plugin)

namespace scrimmage {
namespace controller {

AircraftPIDController::AircraftPIDController() {
}

void AircraftPIDController::init(std::map<std::string, std::string> &params) {
    use_roll_control_ = sc::get<bool>("use_roll_control", params, use_roll_control_);

    // Setup input variables
    if (use_roll_control_) {
        desired_roll_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::In);
    } else {
        desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);

        // Outer loop heading PID
        if (!heading_pid_.init(params["heading_pid"], true)) {
            std::cout << "Failed to set heading PID" << std::endl;
        }
    }
    desired_altitude_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::In);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);

    // Setup output variables
    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::Out);
    aileron_idx_ = vars_.declare(VariableIO::Type::aileron, VariableIO::Direction::Out);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::Out);

    // Outer loop PIDs
    if (!altitude_pid_.init(params["altitude_pid"], false)) {
        std::cout << "Failed to set altitude PID" << std::endl;
    }
    if (!speed_pid_.init(params["speed_pid"], false)) {
        std::cout << "Failed to set speed PID" << std::endl;
    }

    // Inner loop PIDs
    if (!pitch_pid_.init(params["pitch_pid"], true)) {
        std::cout << "Failed to set pitch PID" << std::endl;
    }
    if (!roll_pid_.init(params["roll_pid"], true)) {
        std::cout << "Failed to set roll PID" << std::endl;
    }

    max_pitch_ = sc::Angles::deg2rad(sc::get<double>("max_pitch", params, max_pitch_));
    max_roll_ = sc::Angles::deg2rad(sc::get<double>("max_roll", params, max_roll_));
}

bool AircraftPIDController::step(double t, double dt) {
    // Desired altitude to desired pitch
    altitude_pid_.set_setpoint(vars_.input(desired_altitude_idx_));
    double desired_pitch = -altitude_pid_.step(time_->dt(), state_->pos()(2));
    desired_pitch = clamp(desired_pitch, -max_pitch_, max_pitch_);

    // Desired pitch to elevator
    pitch_pid_.set_setpoint(desired_pitch);
    double elevator = clamp(pitch_pid_.step(time_->dt(), state_->quat().pitch()), -1.0, 1.0);
    vars_.output(elevator_idx_, elevator);

    double desired_roll;
    if (use_roll_control_) {
        desired_roll = vars_.input(desired_roll_idx_);
    } else {
        // Desired heading to desired roll
        heading_pid_.set_setpoint(vars_.input(desired_heading_idx_));
        desired_roll = -heading_pid_.step(time_->dt(), state_->quat().yaw());
    }
    desired_roll = clamp(desired_roll, -max_roll_, max_roll_);

    // Desired roll to aileron
    roll_pid_.set_setpoint(desired_roll);
    double aileron = clamp(roll_pid_.step(time_->dt(), state_->quat().roll()), -1.0, 1.0);
    vars_.output(aileron_idx_, aileron);

    // Speed to throttle PID
    speed_pid_.set_setpoint(vars_.input(desired_speed_idx_));
    double throttle = clamp(speed_pid_.step(time_->dt(), state_->vel().norm()), -1.0, 1.0);
    vars_.output(throttle_idx_, throttle);

    // Rudder is static for now
    vars_.output(rudder_idx_, 0.0);

    return true;
}
} // namespace controller
} // namespace scrimmage
