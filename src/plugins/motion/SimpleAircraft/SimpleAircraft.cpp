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
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/motion/SimpleAircraft/SimpleAircraft.h>

#include <boost/algorithm/clamp.hpp>

using boost::algorithm::clamp;

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::SimpleAircraft, SimpleAircraft_plugin)

namespace scrimmage {
namespace motion {

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
    Quaternion &quat = state_->quat();

    min_velocity_ = get("min_velocity", params, 15.0);
    max_velocity_ = get("max_velocity", params, 40.0);
    max_roll_ = Angles::deg2rad(get("max_roll", params, 30.0));
    max_pitch_ = Angles::deg2rad(get("max_pitch", params, 30.0));
    max_pitch_rate_ = Angles::deg2rad(get("max_pitch_rate", params, 57.3));
    max_roll_rate_ = Angles::deg2rad(get("max_roll_rate", params, 57.3));


    x_[X] = pos(0);
    x_[Y] = pos(1);
    x_[Z] = pos(2);
    x_[ROLL] = clamp(quat.roll(), -max_roll_, max_roll_);
    x_[PITCH] = clamp(quat.pitch(), -max_pitch_, max_pitch_);
    x_[YAW] = quat.yaw();
    x_[SPEED] = clamp(state_->vel().norm(), min_velocity_, max_velocity_);

    length_ = get("turning_radius", params, 50.0);
    speedTarget_ = get("speed_target", params, 50.0);  // The "0" speed for adjusting the turning radius
    lengthSlopePerSpeed_ = get("radius_slope_per_speed", params, 0.0);  // Enables adjusting the turning radius based on speed

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(-x_[ROLL], x_[PITCH], x_[YAW]);
    state_->vel() << x_[SPEED] * cos(x_[5]), x_[SPEED] * sin(x_[5]), 0;

    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::In);
    roll_rate_idx_ = vars_.declare(VariableIO::Type::roll_rate, VariableIO::Direction::In);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::In);

    return true;
}

bool SimpleAircraft::step(double time, double dt) {
    // Need to saturate state variables before model runs
    x_[ROLL] = clamp(x_[ROLL], -max_roll_, max_roll_);
    x_[PITCH] = clamp(x_[PITCH], -max_pitch_, max_pitch_);
    x_[SPEED] = clamp(x_[SPEED], min_velocity_, max_velocity_);

    ode_step(dt);

    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(-x_[ROLL], x_[PITCH], x_[YAW]);
    state_->vel() << x_[SPEED] * cos(x_[YAW]) * cos(x_[PITCH]), x_[SPEED] * sin(x_[YAW]) * cos(x_[PITCH]), x_[SPEED] * sin(x_[PITCH]);

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
    double throttle = vars_.input(throttle_idx_);
    double roll_rate = vars_.input(roll_rate_idx_);
    double pitch_rate = vars_.input(pitch_rate_idx_);

    // Saturate control inputs
    throttle = clamp(throttle, -100.0, 100.0);
    roll_rate = clamp(roll_rate, -max_roll_rate_, max_roll_rate_);
    pitch_rate = clamp(pitch_rate, -max_pitch_rate_, max_pitch_rate_);

    double xy_speed = x[SPEED] * cos(x[PITCH]);
    dxdt[X] = xy_speed*cos(x[YAW]);
    dxdt[Y] = xy_speed*sin(x[YAW]);
    dxdt[Z] = -sin(x[PITCH])*x[SPEED];
    dxdt[ROLL] = roll_rate;
    dxdt[PITCH] = pitch_rate;
    // Adjust the length based on the speed
    double currentLength = length_ + lengthSlopePerSpeed_ * (x[SPEED] - speedTarget_);
    dxdt[YAW] = x[SPEED]/currentLength*tan(x[ROLL]);

    dxdt[SPEED] = throttle/5;
}

void SimpleAircraft::teleport(StatePtr &state) {
    x_[X] = state->pos()[0];
    x_[Y] = state->pos()[1];
    x_[Z] = state->pos()[2];
    x_[ROLL] = state->quat().roll();
    x_[PITCH] = state->quat().pitch();
    x_[YAW] = state->quat().yaw();
    x_[SPEED] = state->vel()[0];
}
}  // namespace motion
}  // namespace scrimmage
