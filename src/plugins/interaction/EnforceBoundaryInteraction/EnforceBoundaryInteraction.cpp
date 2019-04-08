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

#include <scrimmage/plugins/interaction/EnforceBoundaryInteraction/EnforceBoundaryInteraction.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/plugins/interaction/Boundary/Boundary.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sci = scrimmage::interaction;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::EnforceBoundaryInteraction,
                EnforceBoundaryInteraction_plugin)

namespace scrimmage {
namespace interaction {

EnforceBoundaryInteraction::EnforceBoundaryInteraction() {
}

bool EnforceBoundaryInteraction::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {

    std::vector<int> active_boundary_ids;
    if (!sc::get_vec<int>("active_boundary_ids", plugin_params, ", ", active_boundary_ids)) {
        std::cout << "Failed to parse 'active_boundary_ids'" << endl;
        return false;
    } else {
        active_boundary_ids_ = std::set<int>(active_boundary_ids.begin(),
                                             active_boundary_ids.end());
    }

    auto callback = [&] (scrimmage::MessagePtr<sp::Shape> msg) {
        if (active_boundary_ids_.count(msg->data.id().id()) != 0) {
            boundaries_.push_back(sci::Boundary::make_boundary(msg->data));
        }
    };
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", callback);

    return true;
}


bool EnforceBoundaryInteraction::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                         double t, double dt) {

    // If the boundaries haven't been received yet, ignore, just return.
    if (boundaries_.empty()) return true;

    // The entity must be within at least one of the active boundaries,
    // otherwise, it is removed from the simulation
    for (sc::EntityPtr ent : ents) {
        bool within_a_boundary = false;
        for (std::shared_ptr<sci::BoundaryBase> boundary : boundaries_) {
            if (boundary->contains(ent->state_truth()->pos())) {
                within_a_boundary = true;
            }
        }
        if (!within_a_boundary) {
            ent->collision();
        }
    }
    return true;
}
} // namespace interaction
} // namespace scrimmage
