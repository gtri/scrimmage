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

#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/JSBSimModelControllerDirect/JSBSimModelControllerDirect.h>

#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::JSBSimModelControllerDirect, JSBSimModelControllerDirect_plugin)

namespace scrimmage {
namespace controller {

void JSBSimModelControllerDirect::init(std::map<std::string, std::string> &params) {
    use_pitch_ = str2bool(params.at("use_pitch"));
    std::string z_name =  use_pitch_ ?
        vars_.type_map().at(VariableIO::Type::desired_pitch) :
        vars_.type_map().at(VariableIO::Type::desired_altitude);

    input_vel_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    input_roll_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::In);
    input_alt_or_pitch_idx_ = vars_.declare(z_name, VariableIO::Direction::In);

    output_vel_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    output_roll_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::Out);
    output_alt_or_pitch_idx_ = vars_.declare(z_name, VariableIO::Direction::Out);
}

bool JSBSimModelControllerDirect::step(double t, double dt) {
    vars_.output(input_vel_idx_, vars_.input(input_vel_idx_));
    vars_.output(input_roll_idx_, vars_.input(input_roll_idx_));
    vars_.output(input_alt_or_pitch_idx_, vars_.input(input_alt_or_pitch_idx_));
    return true;
}
} // namespace controller
} // namespace scrimmage
