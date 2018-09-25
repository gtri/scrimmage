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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/controller/DoubleIntegratorControllerWaypoint/DoubleIntegratorControllerWaypoint.h>

#include <iostream>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::DoubleIntegratorControllerWaypoint, DoubleIntegratorControllerWaypoint_plugin)

namespace scrimmage {
namespace controller {

void DoubleIntegratorControllerWaypoint::init(std::map<std::string, std::string> &params) {
    std::vector<double> gain;
    if (!scrimmage::str2container(params.at("gain"), ",", gain, 2)) {
        std::cout << "warning: did not get gain properly in DoubleIntegratorControllerWaypoint" << std::endl;
    } else {
        gain_ << gain[0], gain[1];
    }

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::In);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::In);

    acc_x_idx_ = vars_.declare(VariableIO::Type::acceleration_x, VariableIO::Direction::Out);
    acc_y_idx_ = vars_.declare(VariableIO::Type::acceleration_y, VariableIO::Direction::Out);
    acc_z_idx_ = vars_.declare(VariableIO::Type::acceleration_z, VariableIO::Direction::Out);
    turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
}

bool DoubleIntegratorControllerWaypoint::step(double t, double dt) {
    Eigen::Vector3d acc = -gain_(0) * (state_->pos() - desired_state_->pos())
        - gain_(1) * state_->vel();

    vars_.output(acc_x_idx_, acc(0));
    vars_.output(acc_y_idx_, acc(1));
    vars_.output(acc_z_idx_, acc(2));
    vars_.output(turn_rate_idx_, 0);

    return true;
}
} // namespace controller
} // namespace scrimmage
