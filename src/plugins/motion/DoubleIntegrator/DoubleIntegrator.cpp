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
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugins/motion/DoubleIntegrator/DoubleIntegrator.h>
#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::MotionModel, DoubleIntegrator, DoubleIntegrator_plugin)

namespace pl = std::placeholders;
using boost::algorithm::clamp;

enum ModelParams {X, Y, Z, VX, VY, VZ, STATE_SIZE};

DoubleIntegrator::DoubleIntegrator() {
    x_.resize(STATE_SIZE);
}

bool DoubleIntegrator::init(std::map<std::string, std::string> &info,
                            std::map<std::string, std::string> &params) {
    max_vel_ = std::stod(params.at("max_vel"));
    max_acc_ = std::stod(params.at("max_acc"));

    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);

    x_[VX] = clamp(state_->vel()(0), -max_vel_, max_vel_);
    x_[VY] = clamp(state_->vel()(1), -max_vel_, max_vel_);
    x_[VZ] = clamp(state_->vel()(2), -max_vel_, max_vel_);

    state_->pos() << x_[X], x_[Y], x_[Z];

    return true;
}

bool DoubleIntegrator::step(double t, double dt) {
    ctrl_u_ = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();
    ctrl_u_(0) = clamp(ctrl_u_(0), -max_acc_, max_acc_);
    ctrl_u_(1) = clamp(ctrl_u_(1), -max_acc_, max_acc_);
    ctrl_u_(2) = clamp(ctrl_u_(2), -max_acc_, max_acc_);

    ode_step(dt);

    x_[VX] = clamp(x_[VX], -max_vel_, max_vel_);
    x_[VY] = clamp(x_[VY], -max_vel_, max_vel_);
    x_[VZ] = clamp(x_[VZ], -max_vel_, max_vel_);

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->vel() << x_[VX], x_[VY], x_[VZ];
    state_->quat().set(0, 0, atan2(x_[VY], x_[VX]));
    return true;
}

double DoubleIntegrator::update_dvdt(double vel, double acc) {
    if (vel >= max_vel_ && acc > 0) {
        return 0;
    } else if (vel <= -max_vel_ && acc < 0) {
        return 0;
    } else {
        return acc;
    }
}

void DoubleIntegrator::model(const vector_t &x , vector_t &dxdt , double t) {
    dxdt[X] = clamp(x_[VX], -max_vel_, max_vel_);
    dxdt[Y] = clamp(x_[VY], -max_vel_, max_vel_);
    dxdt[Z] = clamp(x_[VZ], -max_vel_, max_vel_);

    dxdt[VX] = update_dvdt(x[VX], ctrl_u_(0));
    dxdt[VY] = update_dvdt(x[VY], ctrl_u_(1));
    dxdt[VZ] = update_dvdt(x[VZ], ctrl_u_(2));
}
