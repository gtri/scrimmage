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

#include <(>>>PROJECT_NAME<<<)/plugins/autonomy/ExamplePlugin/ExamplePlugin.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/VariableIO.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::ExamplePlugin,
                ExamplePlugin_plugin)

namespace scrimmage {
namespace autonomy {

ExamplePlugin::ExamplePlugin() : follow_id_(-1) {
}

void ExamplePlugin::init(std::map<std::string, std::string> &params) {
    initial_speed_ = sc::get<double>("initial_speed", params, 21);

    // VariableIO
    desired_altitude_idx_ = vars_.declare("desired_altitude", scrimmage::VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare("desired_heading", scrimmage::VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare("desired_speed", scrimmage::VariableIO::Direction::Out);
}

bool ExamplePlugin::step_autonomy(double t, double dt) {

    // Find nearest entity on other team. Loop through each contact, calculate
    // distance to entity, save the ID of the entity that is closest.
    double min_dist = std::numeric_limits<double>::infinity();
    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {

        // Skip if this contact is on the same team
        if (it->second.id().team_id() == parent_->id().team_id()) {
            continue;
        }

        // Calculate distance to entity
        double dist = (it->second.state()->pos() - state_->pos()).norm();

        if (dist < min_dist) {
            // If this is the minimum distance, save distance and reference to
            // entity
            min_dist = dist;
            follow_id_ = it->first;
        }
    }

    // Head toward entity on other team
    if (contacts_->count(follow_id_) > 0) {
        // Get a reference to the entity's state.
        sc::StatePtr ent_state = contacts_->at(follow_id_).state();

        // Calculate the required heading to follow the other entity
        desired_heading_ = atan2(ent_state->pos()(1) - state_->pos()(1),
                           ent_state->pos()(0) - state_->pos()(0));

        // Match entity's altitude
        desired_altitude_ = ent_state->pos()(2);

				// Match entity's speed
        desired_speed_ = ent_state->vel()(0);
    }

		// set VariableIO desired values
    vars_.output(desired_altitude_idx_, desired_altitude_);
    vars_.output(desired_heading_idx_, desired_heading_);
    vars_.output(desired_speed_idx_, desired_speed_);

    return true;
}
} // namespace autonomy
} // namespace scrimmage
