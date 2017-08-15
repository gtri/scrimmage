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
#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::MotionModel, Unicycle,
                Unicycle_plugin)

namespace sc = scrimmage;

namespace pl = std::placeholders;

enum ModelParams {
    X = 0,
    Y,
    THETA,
    MODEL_NUM_ITEMS
};

bool Unicycle::init(std::map<std::string, std::string> &info,
                            std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[THETA] = state_->quat().yaw();

    turn_rate_max_ = std::stod(params.at("turn_rate_max"));
    vel_max_ = std::stod(params.at("vel_max"));
    return true;
}

bool Unicycle::step(double t, double dt) {
    ctrl_u_ = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();

    double prev_x = x_[X];
    double prev_y = x_[Y];

    ode_step(dt);

    double dx = (x_[X] - prev_x) / dt;
    double dy = (x_[Y] - prev_y) / dt;

    state_->vel()(0) = dx;
    state_->vel()(1) = dy;

    state_->pos()(0) = x_[X];
    state_->pos()(1) = x_[Y];

    state_->quat().set(0, 0, x_[THETA]);

    return true;
}

void Unicycle::model(const vector_t &x , vector_t &dxdt , double t) {
    double vel = boost::algorithm::clamp(ctrl_u_(0), -vel_max_, vel_max_);
    double yaw_rate = boost::algorithm::clamp(ctrl_u_(1), -turn_rate_max_, turn_rate_max_);
    dxdt[X] = vel * cos(x[THETA]);
    dxdt[Y] = vel * sin(x[THETA]);
    dxdt[THETA] = yaw_rate;
}
