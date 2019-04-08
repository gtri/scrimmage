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

#include <scrimmage/plugins/interaction/CaptureInBoundaryInteraction/CaptureInBoundaryInteraction.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Capture.pb.h>

#include <scrimmage/plugins/interaction/Boundary/Boundary.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sci = scrimmage::interaction;
namespace sm = scrimmage_msgs;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::CaptureInBoundaryInteraction,
                CaptureInBoundaryInteraction_plugin)

namespace scrimmage {
namespace interaction {

CaptureInBoundaryInteraction::CaptureInBoundaryInteraction() :
    capture_range_(5.0), boundary_id_(0), cool_down_period_(0.0) {
}

bool CaptureInBoundaryInteraction::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {

    capture_range_ = sc::get<double>("capture_range", plugin_params, 5.0);
    boundary_id_ = sc::get<int>("boundary_id", plugin_params, 1);
    cool_down_period_ = sc::get<double>("cool_down_period", plugin_params, 0.0);

    auto callback = [&] (scrimmage::MessagePtr<sp::Shape> msg) {
        if (msg->data.id().id() == boundary_id_) {
            boundary_shape_ = msg->data;
            boundary_ = sci::Boundary::make_boundary(msg->data);
        }
    };
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", callback);

    non_team_capture_pub_ = advertise("GlobalNetwork", "NonTeamCapture");

    return true;
}


bool CaptureInBoundaryInteraction::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                           double t, double dt) {
    if (boundary_ == nullptr) {
        return true;
    }

    // Key : Possibly captured entity
    // Value: Entity performing the capture
    std::map<int, int> possible_captures;

    // Search over all entities whose team ID is the same as this boundary's
    // team id and are within this boundary.
    for (sc::EntityPtr ent : ents) {
        // Determine if this entity has captured any other entities within the
        // cool down period.
        bool cool_down_expired = true;
        auto it = prev_capture_times_.find(ent->id().id());
        if (it != prev_capture_times_.end()) {
            if (time_->t() <= (it->second + cool_down_period_)) {
                cool_down_expired = false;
            }
        }

        if (cool_down_expired &&
            ent->id().team_id() == boundary_shape_.id().team_id() &&
            boundary_->contains(ent->state_truth()->pos())) {

            // Find all entities within capture range of this entity
            std::vector<ID> rtree_neighbors;
            parent_->rtree()->neighbors_in_range(ent->state_truth()->pos(),
                                                 rtree_neighbors,
                                                 capture_range_,
                                                 ent->id().id());

            // Only copy IDs whose team ID isn't the same as the boundary's
            // team ID.
            for (ID &id : rtree_neighbors) {
                if (id.team_id() != boundary_shape_.id().team_id()) {
                    possible_captures[id.id()] = ent->id().id();
                }
            }
        }
    }

    // For all entities that are possibly captured, determine if they are
    // within the boundary
    for (sc::EntityPtr ent : ents) {
        auto it = possible_captures.find(ent->id().id());
        if (it != possible_captures.end()) {
            // If the entity hasn't been captured yet and it's within the
            // boundary, it is captured
            if (already_captured_.count(it->first) == 0 &&
                boundary_->contains(ent->state_truth()->pos())) {
                ent->collision();

                // Keep track of all captured entities
                already_captured_.insert(it->first);

                // Keep track of the capture time to limit the capture rate
                prev_capture_times_[it->second] = time_->t();

                auto msg =
                    std::make_shared<sc::Message<sm::NonTeamCapture>>();
                msg->data.set_source_id(it->second);
                msg->data.set_target_id(it->first);
                non_team_capture_pub_->publish(msg);
                // std::cout << "ID " << it->second << " captured " << it->first
                //           << std::endl;
            }
        }
    }
    return true;
}
} // namespace interaction
} // namespace scrimmage
