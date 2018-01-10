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

#include <scrimmage/plugins/motion/Unicycle/Unicycle.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <iostream>

#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::Unicycle, Unicycle_plugin)

using boost::algorithm::clamp;

enum ModelParams {
    X = 0,
    Y,
    Z,
    YAW,
    PITCH,
    MODEL_NUM_ITEMS
};

namespace scrimmage {
namespace motion {

bool Unicycle::init(std::map<std::string, std::string> &info,
                    std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);
    x_[YAW] = state_->quat().yaw();
    x_[PITCH] = state_->quat().pitch();

    turn_rate_max_ = std::stod(params.at("turn_rate_max"));
    pitch_rate_max_ = std::stod(params.at("pitch_rate_max"));
    vel_max_ = std::stod(params.at("vel_max"));
    enable_roll_ = get<bool>("enable_roll", params, false);

    return true;
}

bool Unicycle::step(double t, double dt) {
    std::shared_ptr<Controller> ctrl =
        std::dynamic_pointer_cast<Controller>(parent_->controllers().back());
    if (ctrl == nullptr) {
        std::cout << "could not cast unicycle controller" << std::endl;
        return false;
    }

    // save control value and make sure it satisfies constraints
    ctrl_u_ = ctrl->u();
    ctrl_u_(0) = clamp(ctrl_u_(0), -vel_max_, vel_max_);
    ctrl_u_(1) = clamp(ctrl_u_(1), -turn_rate_max_, turn_rate_max_);
    ctrl_u_(2) = clamp(ctrl_u_(2), -pitch_rate_max_, pitch_rate_max_);

    double prev_x = x_[X];
    double prev_y = x_[Y];
    double prev_z = x_[Z];

    ode_step(dt);

    double dx = (x_[X] - prev_x) / dt;
    double dy = (x_[Y] - prev_y) / dt;
    double dz = (x_[Z] - prev_z) / dt;

    state_->vel()(0) = dx;
    state_->vel()(1) = dy;
    state_->vel()(2) = dz;

    state_->pos()(0) = x_[X];
    state_->pos()(1) = x_[Y];
    state_->pos()(2) = x_[Z];

    double roll = 0; // TODO: simulate roll if enabled
    if (enable_roll_ && ctrl_u_(1) != 0) {
        // see https://en.wikipedia.org/wiki/Standard_rate_turn
        const double vel = ctrl_u_(0);
        const double turn_rate = ctrl_u_(1);
        const double radius = vel / turn_rate;
        const double gravity = 9.81;
        roll = atan2(pow(vel, 2) / radius, gravity);
    }
    state_->quat().set(roll, -x_[PITCH], x_[YAW]);

    return true;
}

void Unicycle::model(const vector_t &x , vector_t &dxdt , double t) {
    const double vel = ctrl_u_(0);
    const double yaw_rate = ctrl_u_(1);
    const double pitch_rate = ctrl_u_(2);

    double xy_speed = vel * cos(x[PITCH]);
    dxdt[X] = xy_speed * cos(x[YAW]);
    dxdt[Y] = xy_speed * sin(x[YAW]);
    dxdt[Z] = vel * sin(x[PITCH]);
    dxdt[YAW] = yaw_rate;
    dxdt[PITCH] = pitch_rate;
}
} // namespace motion
} // namespace scrimmage
