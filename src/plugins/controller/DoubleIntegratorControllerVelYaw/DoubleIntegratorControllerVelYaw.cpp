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

void set_pid(sc::PID &pid, const std::string &str, bool is_angle = false) {
    std::vector<std::string> str_vals;
    split(str_vals, str, ",");

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
    vars_.output(acc_z_idx_, alt_u);

    speed_pid_.set_setpoint(vars_.input(desired_speed_idx_));
    double acc_u = speed_pid_.step(dt, state_->vel().norm());

    // Rotate acceleration into forward direction of double integrator
    Eigen::Vector3d acc_vec = state_->quat() * Eigen::Vector3d::UnitX() * acc_u;

    vars_.output(acc_x_idx_, acc_vec(0));
    vars_.output(acc_y_idx_, acc_vec(1));
    vars_.output(acc_z_idx_, acc_vec(2));

    yaw_pid_.set_setpoint(vars_.input(desired_heading_idx_));
    vars_.output(turn_rate_idx_, yaw_pid_.step(dt, state_->quat().yaw()));

    return true;
}
} // namespace controller
} // namespace scrimmage
