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

#include <scrimmage/plugins/controller/UUV6DOFLinearEnergy/UUV6DOFLinearEnergy.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::UUV6DOFLinearEnergy,
                UUV6DOFLinearEnergy_plugin)

namespace scrimmage {
namespace controller {

void UUV6DOFLinearEnergy::init(std::map<std::string, std::string> &params) {
    energy_ = sc::get<double>("energy_initial", params, energy_);
    energy_max_ = sc::get<double>("energy_max", params, energy_max_);
    energy_min_ = sc::get<double>("energy_min", params, energy_min_);

    in_throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::In);
    in_elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::In);
    in_rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::In);

    out_throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);
    out_elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::Out);
    out_rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::Out);
}

bool UUV6DOFLinearEnergy::step(double t, double dt) {
    double throttle = vars_.input(in_throttle_idx_);
    energy_ -= throttle * dt;
    if (energy_ <= energy_min_) {
        energy_ = energy_min_;
        throttle = 0;
    }

    vars_.output(out_throttle_idx_, throttle);
    vars_.output(out_elevator_idx_, vars_.input(in_elevator_idx_));
    vars_.output(out_rudder_idx_, vars_.input(in_rudder_idx_));

    return true;
}
} // namespace controller
} // namespace scrimmage
