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

#include <scrimmage/plugins/controller/UUV6DOFPIDController/UUV6DOFPIDController.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::UUV6DOFPIDController,
                UUV6DOFPIDController_plugin)

namespace scrimmage {
namespace controller {

UUV6DOFPIDController::UUV6DOFPIDController() {
}

void UUV6DOFPIDController::init(std::map<std::string, std::string> &params) {
    desired_altitude_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::In);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);

    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::Out);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::Out);

    if (!heading_pid_.init(params["heading_pid"], true)) {
        std::cout << "Failed to set UUV6DOFPIDController heading_pid" << endl;
    }
    if (!speed_pid_.init(params["speed_pid"], false)) {
        std::cout << "Failed to set UUV6DOFPIDController speed_pid" << endl;
    }
    if (!altitude_pid_.init(params["altitude_pid"], false)) {
        std::cout << "Failed to set UUV6DOFPIDController altitude_pid" << endl;
    }
}

bool UUV6DOFPIDController::step(double t, double dt) {

    heading_pid_.set_setpoint(vars_.input(desired_heading_idx_));
    double u_rudder = heading_pid_.step(time_->dt(), state_->quat().yaw());

    altitude_pid_.set_setpoint(vars_.input(desired_altitude_idx_));
    double u_elevator = -altitude_pid_.step(time_->dt(), state_->pos()(2));

    speed_pid_.set_setpoint(vars_.input(desired_speed_idx_));
    double u_throttle = speed_pid_.step(time_->dt(), state_->vel().norm());

    vars_.output(rudder_idx_, u_rudder);
    vars_.output(elevator_idx_, u_elevator);
    vars_.output(throttle_idx_, u_throttle);

    return true;
}
} // namespace controller
} // namespace scrimmage
