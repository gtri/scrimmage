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
#include <scrimmage/plugins/controller/SingleIntegratorControllerSimple/SingleIntegratorControllerSimple.h>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::SingleIntegratorControllerSimple, SingleIntegratorControllerSimple_plugin)

namespace scrimmage {
namespace controller {

void SingleIntegratorControllerSimple::init(std::map<std::string, std::string> &params) {
    input_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::In);
    input_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::In);
    input_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::In);

    output_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
    output_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
    output_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);
}

bool SingleIntegratorControllerSimple::step(double t, double dt) {
    vars_.output(output_vel_x_idx_, vars_.input(input_vel_x_idx_));
    vars_.output(output_vel_y_idx_, vars_.input(input_vel_y_idx_));
    vars_.output(output_vel_z_idx_, vars_.input(input_vel_z_idx_));
    return true;
}
} // namespace controller
} // namespace scrimmage
