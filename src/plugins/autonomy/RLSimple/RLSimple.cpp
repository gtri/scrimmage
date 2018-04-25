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

#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/ExternalControl.pb.h>

#include <scrimmage/plugins/autonomy/RLSimple/RLSimple.h>

namespace sp = scrimmage_proto;
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, RLSimple, RLSimple_plugin)

void RLSimple::init(std::map<std::string, std::string> &params) {
    output_vel_x_idx_ = vars_.declare(sc::VariableIO::Type::velocity_x, sc::VariableIO::Direction::Out);
    output_vel_y_idx_ = vars_.declare(sc::VariableIO::Type::velocity_y, sc::VariableIO::Direction::Out);
    output_vel_z_idx_ = vars_.declare(sc::VariableIO::Type::velocity_z, sc::VariableIO::Direction::Out);

    radius_ = std::stod(params.at("radius"));
    ExternalControl::init(params);
}

std::pair<bool, double> RLSimple::calc_reward(double t) {
    return {false, state_->pos().head<2>().norm() < radius_};
}

bool RLSimple::handle_action(double t, double dt, const scrimmage_proto::Action &action) {
    if (!check_action(action, 1, 0)) return false;

    double x_vel = action.discrete(0) == 1 ? 1 : -1;
    vars_.output(output_vel_x_idx_, x_vel);
    return true;
}

scrimmage_proto::SpaceParams RLSimple::action_space_params() {
    sp::SpaceParams space_params;
    sp::SingleSpaceParams *single_space_params = space_params.add_params();
    single_space_params->set_discrete(true);
    single_space_params->set_num_dims(1);
    single_space_params->add_minimum(0);
    single_space_params->add_maximum(1);
    return space_params;
}
