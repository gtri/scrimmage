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

#include <scrimmage/plugins/motion/Ballistic/Ballistic.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <boost/algorithm/clamp.hpp>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::MotionModel,
                scrimmage::motion::Ballistic,
                Ballistic_plugin)

using boost::algorithm::clamp;

namespace scrimmage {
namespace motion {

enum ModelParams {
    X = 0,
    Y,
    Z,
    X_vel,
    Y_vel,
    Z_vel,
    X_acc,
    Y_acc,
    Z_acc,
    MODEL_NUM_ITEMS
};

bool Ballistic::init(std::map<std::string, std::string> &info,
                     std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);
    x_[X_vel] = state_->vel()(0);
    x_[Y_vel] = state_->vel()(1);
    x_[Z_vel] = state_->vel()(2);
    x_[X_acc] = 0;
    x_[Y_acc] = 0;
    x_[Z_acc] = 0;

    // No orientation
    state_->quat().set(0, 0, 0);

    return true;
}

bool Ballistic::step(double time, double dt) {
    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);

    x_[X_vel] = state_->vel()(0);
    x_[Y_vel] = state_->vel()(1);
    x_[Z_vel] = state_->vel()(2);

    ode_step(dt);

    state_->pos()(0) = x_[X];
    state_->pos()(1) = x_[Y];
    state_->pos()(2) = x_[Z];

    state_->vel()(0) = x_[X_vel];
    state_->vel()(1) = x_[Y_vel];
    state_->vel()(2) = x_[Z_vel];

    return true;
}

void Ballistic::model(const vector_t &x , vector_t &dxdt ,
                                double t) {
    dxdt[X] = x[X_vel];
    dxdt[Y] = x[Y_vel];
    dxdt[Z] = x[Z_vel];

    dxdt[X_vel] = x[X_acc];
    dxdt[Y_vel] = x[Y_acc];
    dxdt[Z_vel] = x[Z_acc];

    dxdt[X_acc] = 0;
    dxdt[Y_acc] = 0;
    dxdt[Z_acc] = -mass_ * g_;
}
} // namespace motion
} // namespace scrimmage
