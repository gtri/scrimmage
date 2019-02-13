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

#include <scrimmage/plugins/motion/DubinsAirplane/DubinsAirplane.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::DubinsAirplane, DubinsAirplane_plugin)

namespace scrimmage {
namespace motion {

namespace sc = scrimmage;

namespace pl = std::placeholders;

enum ModelParams {
    X = 0,
    Y,
    Z,
    ROLL,
    PITCH,
    YAW,
    MODEL_NUM_ITEMS
};

bool DubinsAirplane::init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params) {

    // Model limits
    speed_max_ = sc::get<double>("speed_max", params, speed_max_);
    speed_min_ = sc::get<double>("speed_min", params, speed_min_);
    pitch_max_ = sc::Angles::deg2rad(sc::get<double>("pitch_max", params, pitch_max_));
    pitch_min_ = sc::Angles::deg2rad(sc::get<double>("pitch_min", params, pitch_min_));
    roll_max_ = sc::Angles::deg2rad(sc::get<double>("roll_max", params, roll_max_));
    roll_min_ = sc::Angles::deg2rad(sc::get<double>("roll_min", params, roll_min_));

    write_csv_ = sc::get<bool>("write_csv", params, false);

    // If directly controlling speed, pitch angle, and roll angle (bank)
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    desired_pitch_idx_ = vars_.declare(VariableIO::Type::desired_pitch, VariableIO::Direction::In);
    desired_roll_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::In);

    // Model initialization
    x_.resize(MODEL_NUM_ITEMS);

    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);
    x_[ROLL] = state_->quat().roll();
    x_[PITCH] = state_->quat().pitch();
    x_[YAW] = state_->quat().yaw();

    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-dubinsairplane-states.csv");

        csv_.set_column_headers(sc::CSV::Headers{"t",
                        "x", "y", "z",
                        "roll", "pitch", "yaw",
                        "speed"});
    }
    return true;
}

bool DubinsAirplane::step(double t, double dt) {
    // Get inputs and saturate
    speed_ = boost::algorithm::clamp(vars_.input(desired_speed_idx_), speed_min_, speed_max_);
    pitch_ = boost::algorithm::clamp(vars_.input(desired_pitch_idx_), pitch_min_, pitch_max_);
    roll_ = boost::algorithm::clamp(vars_.input(desired_roll_idx_), roll_min_, roll_max_);

    x_[X] = state_->pos()(0);
    x_[Y] = state_->pos()(1);
    x_[Z] = state_->pos()(2);

    // Directly set the vehicle's roll and pitch
    x_[ROLL] = roll_;
    x_[PITCH] = pitch_;
    x_[YAW] = state_->quat().yaw();

    double prev_x = x_[X];
    double prev_y = x_[Y];
    double prev_z = x_[Z];

    ode_step(dt);

    // Compute velocity (change in position)
    state_->vel()(0) = (x_[X] - prev_x) / dt;
    state_->vel()(1) = (x_[Y] - prev_y) / dt;
    state_->vel()(2) = (x_[Z] - prev_z) / dt;

    // Update state position
    state_->pos()(0) = x_[X];
    state_->pos()(1) = x_[Y];
    state_->pos()(2) = x_[Z];

    // Update state orientation
    state_->quat().set(x_[ROLL], x_[PITCH], x_[YAW]);

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"x", state_->pos()(0)},
                {"y", state_->pos()(1)},
                {"z", state_->pos()(2)},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"speed", speed_}});
    }
    return true;
}

void DubinsAirplane::model(const vector_t &x , vector_t &dxdt , double t) {
    // The DubinsAirplane equations of motion are from the following paper:
    // Owen, Mark, Randal W. Beard, and Timothy W. McLain. "Implementing dubins
    // airplane paths on fixed-wing uavs." Handbook of unmanned aerial
    // vehicles. Springer, Dordrecht, 2015. 1677-1701.

    double xy_speed = speed_ * cos(x[PITCH]);
    dxdt[X] = xy_speed * cos(x[YAW]);
    dxdt[Y] = xy_speed * sin(x[YAW]);
    dxdt[Z] = -speed_ * sin(x[PITCH]);

    dxdt[ROLL] = 0;
    dxdt[PITCH] = 0;

    if (std::abs(speed_) < std::numeric_limits<double>::epsilon()) {
        dxdt[YAW] = 0;
    } else {
        dxdt[YAW] = -g_ / speed_ * tan(roll_);
    }
}
} // namespace motion
} // namespace scrimmage
