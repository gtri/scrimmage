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

#include <scrimmage/plugins/motion/SimpleQuadrotor/SimpleQuadrotor.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::SimpleQuadrotor, SimpleQuadrotor_plugin)

namespace scrimmage {
namespace motion {

namespace pl = std::placeholders;
namespace sc = scrimmage;
using ang = sc::Angles;

enum ModelParams {
    X = 0,
    Y,
    Z,
    XDOT,
    YDOT,
    ZDOT,
    YAW,
    YAWDOT,
    MODEL_NUM_ITEMS
};

enum ControlParams {
    X_THRUST = 0,
    Y_THRUST,
    Z_THRUST,
    TURN_RATE,
    CONTROL_NUM_ITEMS
};

bool SimpleQuadrotor::init(std::map<std::string, std::string> &info,
                           std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);
    x_[XDOT] = 0;
    x_[YDOT] = 0;
    x_[ZDOT] = 0;
    x_[YAW] = state_->quat().yaw();
    x_[YAWDOT] = 0;

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(0, x_[XDOT] / 100.0, x_[YAW]);
    state_->vel() << x_[XDOT], x_[YDOT], x_[ZDOT];

    max_vel_ = std::stod(params["max_vel"]);
    max_pitch_ = ang::deg2rad(std::stod(params["max_pitch"]));

    return true;
}

bool SimpleQuadrotor::step(double time, double dt) {
    ode_step(dt);

    state_->pos() << x_[X], x_[Y], x_[Z];

    double pct_of_max_vel = std::min(1.0, x_[XDOT] / max_vel_);
    double pitch = max_pitch_ * pct_of_max_vel;
    state_->quat().set(0, pitch, x_[YAW]);
    state_->vel() << x_[XDOT] * cos(x_[YAW]), x_[XDOT] * sin(x_[YAW]), x_[ZDOT];

    return true;
}

void SimpleQuadrotor::model(const vector_t &x , vector_t &dxdt , double t) {
    /// 0 : x-position
    /// 1 : y-position
    /// 2 : z-position
    /// 3 : x-velocity
    /// 4 : y-velocity
    /// 5 : z-velocity
    /// 6 : yaw
    /// 7 : yaw-velocity

    Eigen::Vector4d &u = std::static_pointer_cast<Controller>(parent_->controllers().back())->u();
    double max_x_velocity = 15;
    double max_z_velocity = 10;

    double x_thrust = u[X_THRUST];
    double y_thrust = u[Y_THRUST];
    double z_thrust = u[Z_THRUST];
    double turn_force = u[TURN_RATE];

    double x_velocity = x[XDOT];
    if (x_velocity > max_x_velocity) {
        x_velocity = max_x_velocity;
    }

    double z_velocity = x[ZDOT];
    if (z_velocity > max_z_velocity) {
        z_velocity = max_z_velocity;
    }

    dxdt[X]      = x_velocity * cos(x[YAW]);
    dxdt[Y]      = x_velocity * sin(x[YAW]);
    dxdt[Z]      = z_velocity;
    dxdt[XDOT]   = x_thrust;
    dxdt[YDOT]   = y_thrust;
    dxdt[ZDOT]   = z_thrust;
    dxdt[YAW]    = x[YAWDOT];
    dxdt[YAWDOT] = turn_force;
}
} // namespace motion
} // namespace scrimmage
