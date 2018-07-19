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
#include <scrimmage/plugins/controller/DoubleIntegratorControllerVelYaw/DoubleIntegratorControllerVelYaw.h>

#include <iostream>
using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::DoubleIntegratorControllerVelYaw, DoubleIntegratorControllerVelYaw_plugin)

namespace scrimmage {
namespace controller {

void DoubleIntegratorControllerVelYaw::init(std::map<std::string, std::string> &params) {
    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::In);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);

    acc_x_idx_ = vars_.declare(VariableIO::Type::acceleration_x, VariableIO::Direction::Out);
    acc_y_idx_ = vars_.declare(VariableIO::Type::acceleration_y, VariableIO::Direction::Out);
    acc_z_idx_ = vars_.declare(VariableIO::Type::acceleration_z, VariableIO::Direction::Out);
    turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);

    if (!sc::set_pid_gains(yaw_pid_, params["yaw_pid"], true)) {
        cout << "Failed to set DoubleIntegratorControllerVewYaw yaw gains"
             << endl;
    }

    if (!sc::set_pid_gains(speed_pid_, params["speed_pid"])) {
        cout << "Failed to set DoubleIntegratorControllerVewYaw speed pid"
             << endl;
    }

    if (!sc::set_pid_gains(alt_pid_, params["alt_pid"])) {
        cout << "Failed to set DoubleIntegratorControllerVewYaw alt pid"
             << endl;
    }
}

bool DoubleIntegratorControllerVelYaw::step(double t, double dt) {
    alt_pid_.set_setpoint(vars_.input(desired_alt_idx_));
    double alt_u = alt_pid_.step(dt, state_->pos()(2));

    double heading = vars_.input(desired_heading_idx_);
    speed_pid_.set_setpoint(vars_.input(desired_speed_idx_));
    double acc_u = speed_pid_.step(dt, state_->vel().norm());
    Eigen::Vector2d dir;
    dir << cos(heading), sin(heading);

    // Rotate acceleration into forward direction of double integrator
    Eigen::Vector2d acc_vec = dir * acc_u;

    vars_.output(acc_x_idx_, acc_vec(0));
    vars_.output(acc_y_idx_, acc_vec(1));
    vars_.output(acc_z_idx_, alt_u);

    yaw_pid_.set_setpoint(vars_.input(desired_heading_idx_));
    double yaw_rate = yaw_pid_.step(dt, state_->quat().yaw());
    vars_.output(turn_rate_idx_, yaw_rate);

    return true;
}
} // namespace controller
} // namespace scrimmage
