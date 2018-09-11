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

#include <(>>>PROJECT_NAME<<<)/plugins/autonomy/(>>>PLUGIN_NAME<<<)/(>>>PLUGIN_NAME<<<).h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::(>>>PLUGIN_NAME<<<),
                (>>>PLUGIN_NAME<<<)_plugin)

namespace scrimmage {
namespace autonomy {

void (>>>PLUGIN_NAME<<<)::init(std::map<std::string, std::string> &params) {
    initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
}

bool (>>>PLUGIN_NAME<<<)::step_autonomy(double t, double dt) {
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
        double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                               ent_state->pos()(0) - state_->pos()(0));
        vars_.output(desired_heading_idx_, heading);

        // Match entity's altitude
        vars_.output(desired_alt_idx_, ent_state->pos()(2));

        // Maintain speed
        vars_.output(desired_speed_idx_, initial_speed_);
    }
    return true;
}
} // namespace autonomy
} // namespace scrimmage
