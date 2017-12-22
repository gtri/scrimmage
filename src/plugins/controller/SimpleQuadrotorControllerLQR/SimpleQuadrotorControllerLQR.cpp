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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/SimpleQuadrotorControllerLQR/SimpleQuadrotorControllerLQR.h>

#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::SimpleQuadrotorControllerLQR, SimpleQuadrotorControllerLQR_plugin)

namespace scrimmage {
namespace controller {

namespace sc = scrimmage;

void SimpleQuadrotorControllerLQR::init(std::map<std::string, std::string> &params) {
    double p_gain = sc::get("vel_p_gain", params, 1.0);
    double i_gain = sc::get("vel_i_gain", params, 1.0);
    double d_gain = sc::get("vel_d_gain", params, 1.0);
    double i_lim = sc::get("i_lim", params, 1.0);
    vel_pid_.set_parameters(p_gain, i_gain, d_gain);
    vel_pid_.set_integral_band(i_lim);

    max_vel_ = std::stod(params["max_vel"]);
}

bool SimpleQuadrotorControllerLQR::step(double t, double dt) {
    Eigen::Vector3d &des_pos = desired_state_->pos();
    double des_yaw = desired_state_->quat().yaw();

    Eigen::Vector3d &pos = state_->pos();
    Eigen::Vector3d &vel = state_->vel();
    double yaw = state_->quat().yaw();
    double xy_speed = vel.head<2>().norm();

    double yaw_dot =
        std::isnan(prev_yaw_) ? 0 : sc::Angles::angle_diff_rad(yaw, prev_yaw_) / dt;
    prev_yaw_ = yaw;

    // LQR Altitude Controller:
    double q1 = 1;
    double z_thrust = -1.0 / q1 * (pos(2) - des_pos(2)) - sqrt(2.0/q1) * vel(2);

    // LQR Heading Controller:
    double q2 = 1;
    double turn_force = -1.0 / q2 * sc::Angles::angle_pi(yaw - des_yaw) - sqrt(2.0/q2) * yaw_dot;

    // If not close to x/y position, use forward velocity:
    double dist = (des_pos - pos).head<2>().norm();
    vel_pid_.set_setpoint((dist < 10) ? 0 : dist / 10);

    double ctrl_vel = vel_pid_.step(dt, xy_speed);

    u_(0) = boost::algorithm::clamp(ctrl_vel, -max_vel_, max_vel_);
    u_(1) = 0;
    u_(2) = z_thrust;
    u_(3) = turn_force;

    return true;
}
} // namespace controller
} // namespace scrimmage
