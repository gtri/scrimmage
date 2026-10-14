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

#include "scrimmage/plugins/motion/SimpleAircraft/SimpleAircraft.h"

#include <boost/algorithm/clamp.hpp>

#include "scrimmage/common/Utilities.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/math/Angles.h"
#include "scrimmage/math/State.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/RegisterPlugin.h"

using boost::algorithm::clamp;

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::SimpleAircraft, SimpleAircraft_plugin)

namespace scrimmage {
namespace motion {

enum ModelParams { X = 0, Y, Z, ROLL, PITCH, YAW, SPEED, MODEL_NUM_ITEMS };

enum ControlParams { THRUST = 0, TURN_RATE, PITCH_RATE, CONTROL_NUM_ITEMS };

std::tuple<int, int, int> SimpleAircraft::version() {
    return std::tuple<int, int, int>(0, 0, 1);
}

bool SimpleAircraft::init(
    std::map<std::string, std::string>& info,
    std::map<std::string, std::string>& params) {
    x_.resize(MODEL_NUM_ITEMS);

    min_velocity_ = get("min_velocity", params, 15.0);
    max_velocity_ = get("max_velocity", params, 40.0);
    max_roll_ = Angles::deg2rad(get("max_roll", params, 30.0));
    max_pitch_ = Angles::deg2rad(get("max_pitch", params, 30.0));
    max_pitch_rate_ = Angles::deg2rad(get("max_pitch_rate", params, 57.3));
    max_roll_rate_ = Angles::deg2rad(get("max_roll_rate", params, 57.3));

    length_ = get("turning_radius", params, 50.0);
    speedTarget_ =
        get("speed_target", params, 50.0);  // The "0" speed for adjusting the turning radius
    lengthSlopePerSpeed_ =
        get("radius_slope_per_speed",
            params,
            0.0);  // Enables adjusting the turning radius based on speed

    // Start within the limits, with velocity along the heading and climb
    // angle, at rest (no commanded rates yet).
    read_truth();
    limit_state();
    write_truth(0, 0, 0);

    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::In);
    roll_rate_idx_ = vars_.declare(VariableIO::Type::roll_rate, VariableIO::Direction::In);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::In);

    return true;
}

bool SimpleAircraft::step(double time, double dt) {
    // Integrate from truth, so a change another plugin made to it (an
    // interaction's push) carries into this step.
    read_truth();
    limit_state();

    ode_step(dt);

    // A step that reaches a limit can pass it before the derivative holds;
    // the limits apply again so truth never holds a state beyond them.
    limit_state();
    write_truth(throttle_input(), roll_rate_input(), pitch_rate_input());

    return true;
}

void SimpleAircraft::read_truth() {
    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);
    x_[ROLL] = -state_->quat().roll();
    x_[PITCH] = state_->quat().pitch_safe();
    x_[YAW] = state_->quat().yaw();
    x_[SPEED] = state_->vel().norm();
}

void SimpleAircraft::limit_state() {
    x_[ROLL] = clamp(x_[ROLL], -max_roll_, max_roll_);
    x_[PITCH] = clamp(x_[PITCH], -max_pitch_, max_pitch_);
    x_[SPEED] = clamp(x_[SPEED], min_velocity_, max_velocity_);
}

double SimpleAircraft::throttle_input() {
    return clamp(vars_.input(throttle_idx_), -100.0, 100.0);
}

double SimpleAircraft::roll_rate_input() {
    return clamp(vars_.input(roll_rate_idx_), -max_roll_rate_, max_roll_rate_);
}

double SimpleAircraft::pitch_rate_input() {
    return clamp(vars_.input(pitch_rate_idx_), -max_pitch_rate_, max_pitch_rate_);
}

void SimpleAircraft::write_truth(double throttle, double roll_rate, double pitch_rate) {
    vector_t dxdt(MODEL_NUM_ITEMS);
    derivative(x_, dxdt, throttle, roll_rate, pitch_rate);
    state_->pos() << x_[X], x_[Y], x_[Z];
    state_->quat().set(-x_[ROLL], x_[PITCH], x_[YAW]);
    state_->vel() << dxdt[X], dxdt[Y], dxdt[Z];
    // Euler rates in world axes, for attitude = yaw * pitch * roll: yaw turns
    // about up, pitch about the yawed y axis, roll about the nose.
    double sin_yaw = sin(x_[YAW]);
    double cos_yaw = cos(x_[YAW]);
    double sin_pitch = sin(x_[PITCH]);
    double cos_pitch = cos(x_[PITCH]);
    double truth_roll_rate = -dxdt[ROLL];
    state_->ang_vel() = Eigen::Vector3d(0, 0, dxdt[YAW])
                        + Eigen::Vector3d(-sin_yaw, cos_yaw, 0) * dxdt[PITCH]
                        + Eigen::Vector3d(cos_yaw * cos_pitch, sin_yaw * cos_pitch, -sin_pitch)
                              * truth_roll_rate;
}

void SimpleAircraft::model(const vector_t& x, vector_t& dxdt, double t) {
    derivative(x, dxdt, throttle_input(), roll_rate_input(), pitch_rate_input());
}

void SimpleAircraft::derivative(
    const vector_t& x, vector_t& dxdt, double throttle, double roll_rate, double pitch_rate) {
    /// 0 : x-position
    /// 1 : y-position
    /// 2 : z-position
    /// 3 : roll
    /// 4 : pitch
    /// 5 : yaw
    /// 6 : speed
    double xy_speed = x[SPEED] * cos(x[PITCH]);
    dxdt[X] = xy_speed * cos(x[YAW]);
    dxdt[Y] = xy_speed * sin(x[YAW]);
    dxdt[Z] = -sin(x[PITCH]) * x[SPEED];
    // Adjust the length based on the speed
    double currentLength = length_ + lengthSlopePerSpeed_ * (x[SPEED] - speedTarget_);
    dxdt[YAW] = x[SPEED] / currentLength * tan(x[ROLL]);

    // A value at a limit stays there while the command pushes it outward.
    // "At" allows for rounding: truth stores attitude as a quaternion, and
    // reading an angle back from it can land a bit inside the limit (about
    // 1e-16 rad); 1e-9 is far above that and far below any motion.
    const double at_limit = 1e-9;
    auto held = [at_limit](double value, double min, double max, double rate) {
        return (value >= max - at_limit && rate > 0.0) || (value <= min + at_limit && rate < 0.0)
                   ? 0.0
                   : rate;
    };
    dxdt[ROLL] = held(x[ROLL], -max_roll_, max_roll_, roll_rate);
    dxdt[PITCH] = held(x[PITCH], -max_pitch_, max_pitch_, pitch_rate);
    dxdt[SPEED] = held(x[SPEED], min_velocity_, max_velocity_, throttle / 5);
}

void SimpleAircraft::teleport(StatePtr& state) {
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
