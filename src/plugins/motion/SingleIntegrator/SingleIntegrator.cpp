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
#include <scrimmage/math/State.h>
#include <scrimmage/entity/Entity.h>

#include <scrimmage/plugins/motion/SingleIntegrator/SingleIntegrator.h>

REGISTER_PLUGIN(scrimmage::MotionModel, SingleIntegrator,
                SingleIntegrator_plugin)

namespace sc = scrimmage;

namespace pl = std::placeholders;

enum ModelParams {
    X = 0,
    Y,
    Z,
    HEADING,
    PITCH,
    MODEL_NUM_ITEMS
};

SingleIntegrator::SingleIntegrator() {
    x_.resize(MODEL_NUM_ITEMS);
}

bool SingleIntegrator::init(std::map<std::string, std::string> &info,
                            std::map<std::string, std::string> &params) {
    x_[X] = std::stod(info["x"]);
    x_[Y] = std::stod(info["y"]);
    x_[Z] = std::stod(info["z"]);

    x_[HEADING] = sc::Angles::deg2rad(std::stod(info["heading"]));
    x_[PITCH] = 0;

    state_->vel() << 0, 0, 0;
    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, x_[PITCH], x_[HEADING]);

    return true;
}

bool SingleIntegrator::step(double t, double dt) {
    ctrl_u_ = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();

    double prev_x = x_[X];
    double prev_y = x_[Y];
    double prev_z = x_[Z];

    ode_step(dt);

    Eigen::Vector3d &vel = state_->vel();
    double dx = (x_[X] - prev_x) / dt;
    double dy = (x_[Y] - prev_y) / dt;
    double dz = (x_[Z] - prev_z) / dt;

    vel << dx, dy, dz;
    state_->pos() << x_[X], x_[Y], x_[Z];

    Eigen::Vector2d xy(dx, dy);

    double yaw = atan2(dy, dx);
    double pitch = atan2(dz, xy.norm());
    state_->quat().set(0, pitch, yaw);

    return true;
}

void SingleIntegrator::model(const vector_t &x , vector_t &dxdt , double t) {
    dxdt[X] = ctrl_u_(X);
    dxdt[Y] = ctrl_u_(Y);
    dxdt[Z] = ctrl_u_(Z);
}
