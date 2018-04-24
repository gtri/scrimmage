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
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/SimpleCarControllerHeading/SimpleCarControllerHeading.h>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::SimpleCarControllerHeading, SimpleCarControllerHeading_plugin)

namespace scrimmage {
namespace controller {

void SimpleCarControllerHeading::init(std::map<std::string, std::string> &params) {
    double p_gain = get<double>("p_gain", params, 1);
    double i_gain = get<double>("i_gain", params, 1);
    double d_gain = get<double>("d_gain", params, 1);
    double i_lim = get<double>("i_lim", params, 1);
    pid_.set_parameters(p_gain, i_gain, d_gain);
    pid_.set_integral_band(i_lim);
    pid_.set_is_angle(true);

    input_vel_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    input_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);

    output_vel_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::Out);
    output_turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
}

bool SimpleCarControllerHeading::step(double t, double dt) {
    vars_.output(output_vel_idx_, vars_.input(input_vel_idx_));
    pid_.set_setpoint(vars_.input(input_heading_idx_));
    const double turn_rate = pid_.step(dt, state_->quat().yaw());
    vars_.output(output_turn_rate_idx_, turn_rate);

    return true;
}
} // namespace controller
} // namespace scrimmage
