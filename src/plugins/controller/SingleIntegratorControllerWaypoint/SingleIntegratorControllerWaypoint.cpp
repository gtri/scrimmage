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

#include <scrimmage/common/VariableIO.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/SingleIntegratorControllerWaypoint/SingleIntegratorControllerWaypoint.h>
#include <scrimmage/math/State.h>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::SingleIntegratorControllerWaypoint, SingleIntegratorControllerWaypoint_plugin)

namespace scrimmage {
namespace controller {

void SingleIntegratorControllerWaypoint::init(std::map<std::string, std::string> &params) {
    input_pos_x_idx_ = vars_.declare(VariableIO::Type::position_x, VariableIO::Direction::In);
    input_pos_y_idx_ = vars_.declare(VariableIO::Type::position_y, VariableIO::Direction::In);
    input_pos_z_idx_ = vars_.declare(VariableIO::Type::position_z, VariableIO::Direction::In);

    output_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
    output_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
    output_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);

    gain_ = std::stod(params.at("gain"));
}

bool SingleIntegratorControllerWaypoint::step(double t, double dt) {
    const Eigen::Vector3d &p = state_->pos();

    const double des_x = vars_.input(input_pos_x_idx_);
    const double des_y = vars_.input(input_pos_y_idx_);
    const double des_z = vars_.input(input_pos_z_idx_);

    vars_.output(output_vel_x_idx_, gain_ * (des_x - p(0)));
    vars_.output(output_vel_y_idx_, gain_ * (des_y - p(1)));
    vars_.output(output_vel_z_idx_, gain_ * (des_z - p(2)));

    return true;
}

} // namespace controller
} // namespace scrimmage
