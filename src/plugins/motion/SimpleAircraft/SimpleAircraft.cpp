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

#include <scrimmage/plugins/motion/SimpleAircraft/SimpleAircraft.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/entity/Entity.h>

#include <boost/algorithm/clamp.hpp>

using boost::algorithm::clamp;

REGISTER_PLUGIN(scrimmage::MotionModel, SimpleAircraft, SimpleAircraft_plugin)

namespace sc = scrimmage;
namespace pl = std::placeholders;

enum ModelParams {
    X = 0,
    Y,
    Z,
    ROLL,
    PITCH,
    YAW,
    SPEED,
    MODEL_NUM_ITEMS
};

enum ControlParams {
    THRUST = 0,
    TURN_RATE,
    PITCH_RATE,
    CONTROL_NUM_ITEMS
};

std::tuple<int, int, int> SimpleAircraft::version() {
    return std::tuple<int, int, int>(0, 0, 1);
}

bool SimpleAircraft::init(std::map<std::string, std::string> &info,
                          std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    Eigen::Vector3d &pos = state_->pos();
    sc::Quaternion &quat = state_->quat();

    x_[X] = pos(0);
    x_[Y] = pos(1);
    x_[Z] = pos(2);
    x_[ROLL] = quat.roll();
    x_[PITCH] = quat.pitch();
    x_[YAW] = quat.yaw();
    x_[SPEED] = state_->vel().norm();

    length_ = sc::get("turning_radius", params, 50.0);

    min_velocity_ = sc::get("min_velocity", params, 15.0);
    max_velocity_ = sc::get("max_velocity", params, 40.0);
    max_roll_ = sc::Angles::deg2rad(sc::get("max_roll", params, 30.0));
    max_pitch_ = sc::Angles::deg2rad(sc::get("max_pitch", params, 30.0));

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(-x_[ROLL], x_[PITCH], x_[YAW]);
    state_->vel() << x_[SPEED] * cos(x_[5]), x_[SPEED] * sin(x_[5]), 0;

    return true;
}

bool SimpleAircraft::step(double time, double dt) {
    // Need to saturate state variables before model runs
    x_[ROLL] = clamp(x_[ROLL], -max_roll_, max_roll_);
    x_[PITCH] = clamp(x_[PITCH], -max_pitch_, max_pitch_);
    x_[SPEED] = clamp(x_[SPEED], min_velocity_, max_velocity_);

    if (ctrl_u_ == nullptr) {
        std::shared_ptr<Controller> ctrl =
            std::dynamic_pointer_cast<Controller>(parent_->controllers().back());
        if (ctrl) {
            ctrl_u_ = ctrl->u();
        }
    }

    if (ctrl_u_ == nullptr) {
        return false;
    }

    ode_step(dt);

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(-x_[ROLL], x_[PITCH], x_[YAW]);
    state_->vel() << x_[SPEED] * cos(x_[YAW]), x_[SPEED] * sin(x_[YAW]), 0;

    return true;
}

void SimpleAircraft::model(const vector_t &x , vector_t &dxdt , double t) {
    /// 0 : x-position
    /// 1 : y-position
    /// 2 : z-position
    /// 3 : roll
    /// 4 : pitch
    /// 5 : yaw
    /// 6 : speed
    double thrust = (*ctrl_u_)(THRUST);
    double roll_rate = (*ctrl_u_)(TURN_RATE);
    double pitch_rate = (*ctrl_u_)(PITCH_RATE);

    // Saturate control inputs
    thrust = clamp(thrust, -100.0, 100.0);
    roll_rate = clamp(roll_rate, -1, 1.0);
    pitch_rate = clamp(pitch_rate, -1.0, 1.0);

    double xy_speed = x[SPEED] * cos(x[PITCH]);
    dxdt[X] = xy_speed*cos(x[YAW]);
    dxdt[Y] = xy_speed*sin(x[YAW]);
    dxdt[Z] = -sin(x[PITCH])*x[SPEED];
    dxdt[ROLL] = roll_rate/2;
    dxdt[PITCH] = pitch_rate/2;
    dxdt[YAW] = x[SPEED]/length_*tan(x[ROLL]);

    dxdt[SPEED] = thrust/5;
}

void SimpleAircraft::teleport(sc::StatePtr &state) {
    x_[X] = state->pos()[0];
    x_[Y] = state->pos()[1];
    x_[Z] = state->pos()[2];
    x_[ROLL] = state->quat().roll();
    x_[PITCH] = state->quat().pitch();
    x_[YAW] = state->quat().yaw();
    x_[SPEED] = state->vel()[0];
}
