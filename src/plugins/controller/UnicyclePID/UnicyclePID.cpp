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

#include <scrimmage/plugins/controller/UnicyclePID/UnicyclePID.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Time.h>

#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::UnicyclePID,
                UnicyclePID_plugin)

namespace scrimmage {
namespace controller {

UnicyclePID::UnicyclePID() {
}

void UnicyclePID::init(std::map<std::string, std::string> &params) {
    show_shapes_ = sc::get<bool>("show_shapes", params, show_shapes_);

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::In);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);

    // Is the motion model using speed or acceleration input control?
    std::string input_name = vars_.type_map().at(VariableIO::Type::speed);
    if (vars_.exists(VariableIO::Type::acceleration_x, VariableIO::Direction::Out)) {
        input_name = vars_.type_map().at(VariableIO::Type::acceleration_x);
        use_accel_ = true;
    }

    speed_idx_ = vars_.declare(input_name, VariableIO::Direction::Out);

    turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);
    roll_rate_idx_ = vars_.declare(VariableIO::Type::roll_rate, VariableIO::Direction::Out);

    // Outer loop PIDs
    if (!heading_pid_.init(params["heading_pid"], true)) {
        std::cout << "Failed to set heading PID" << std::endl;
    }
    if (!pitch_pid_.init(params["pitch_pid"], false)) {
        std::cout << "Failed to set pitch PID" << std::endl;
    }
    if (!roll_pid_.init(params["roll_pid"], false)) {
        std::cout << "Failed to set roll PID" << std::endl;
    }
    if (!speed_pid_.init(params["speed_pid"], false)) {
        std::cout << "Failed to set speed PID" << std::endl;
    }
}

bool UnicyclePID::step(double t, double dt) {
    heading_pid_.set_setpoint(vars_.input(desired_heading_idx_));
    vars_.output(turn_rate_idx_, heading_pid_.step(time_->dt(), state_->quat().yaw()));

    // Reconstruct original velocity vector (close, but not exact
    double x_vel = vars_.input(desired_speed_idx_) / sqrt(1 + pow(tan(vars_.input(desired_heading_idx_)), 2));
    double y_vel = x_vel * tan(vars_.input(desired_heading_idx_));
    Eigen::Vector3d vel(x_vel, y_vel, vars_.input(desired_alt_idx_) - state_->pos()(2));
    vel = vel.normalized() * vars_.input(desired_speed_idx_);

    // Track desired pitch
    double desired_pitch = atan2(vel(2), vel.head<2>().norm());
    pitch_pid_.set_setpoint(desired_pitch);
    vars_.output(pitch_rate_idx_, -pitch_pid_.step(time_->dt(), - state_->quat().pitch()));

    // Track zero roll
    roll_pid_.set_setpoint(0);
    vars_.output(roll_rate_idx_, -roll_pid_.step(time_->dt(), - state_->quat().roll()));

    if (show_shapes_) {
        line_shape_->set_persistent(true);
        sc::set(line_shape_->mutable_color(), 255, 0, 0);
        line_shape_->set_opacity(1.0);
        sc::set(line_shape_->mutable_line()->mutable_start(), state_->pos());
        sc::set(line_shape_->mutable_line()->mutable_end(), vel + state_->pos());
        draw_shape(line_shape_);
    }

    // Only use PID if controlling acceleration, otherwise, we directly set the
    // forward speed
    double desired_speed = vars_.input(desired_speed_idx_);
    if (use_accel_) {
        speed_pid_.set_setpoint(desired_speed);
        desired_speed = speed_pid_.step(time_->dt(), state_->vel().norm());
    }
    vars_.output(speed_idx_, desired_speed);

    return true;
}
} // namespace controller
} // namespace scrimmage
