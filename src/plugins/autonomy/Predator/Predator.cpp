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

#include <iostream>
#include <limits>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/Capture.pb.h>

#include <scrimmage/plugins/autonomy/Predator/Predator.h>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Autonomy, Predator, Predator_plugin)

Predator::Predator()
{
}

void Predator::init(std::map<std::string,std::string> &params)
{
    max_speed_ = sc::get<double>("max_speed", params, 21);
    capture_range_ = sc::get<double>("capture_range", params, 5);

    capture_ent_pub_ = create_publisher("CaptureEntity");

    follow_id_ = -1;

    desired_state_->vel() = Eigen::Vector3d::UnitX()*max_speed_;
    desired_state_->quat().set(0,0,state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);
}

bool Predator::step_autonomy(double t, double dt)
{
    if (contacts_->count(follow_id_) == 0) {
        follow_id_ = -1;
    }

    if (follow_id_ < 0) {
        // Find nearest entity on other team. Loop through each contact, calculate
        // distance to entity, save the ID of the entity that is closest.
        double min_dist = std::numeric_limits<double>::infinity();
        for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
            // Skip if this contact is on the same team
            if (it->second.id().team_id() == parent_->id().team_id()) continue;

            // Calculate distance to entity
            double dist = (it->second.state()->pos() - state_->pos()).norm();
            if (dist < min_dist) {
                // If this is the minimum distance, save distance and reference
                // to entity
                min_dist = dist;
                follow_id_ = it->first;
            }
        }
    }

    // If any non-team members are within capture range, publish capture
    // message
    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
        // Skip if this contact is on the same team
        if (it->second.id().team_id() == parent_->id().team_id()) continue;

        // Calculate distance to entity
        double dist = (it->second.state()->pos() - state_->pos()).norm();
        // Publish capture message if within capture range
        if (dist < capture_range_) {
            auto msg = std::make_shared<sc::Message<sm::CaptureEntity>>();
            msg->data.set_source_id(parent_->id().id());
            msg->data.set_target_id(it->second.id().id());
            capture_ent_pub_->publish(msg, t);
        }
    }

    // Head toward entity on other team
    if (contacts_->count(follow_id_) > 0) {
        // Get a reference to the entity's state.
        sc::StatePtr ent_state = contacts_->at(follow_id_).state();

        //desired_state_->vel() << Eigen::Vector3d::UnitX()*(ent_state->vel().norm()+1);

        // Calculate the required heading to follow the other entity
        double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                               ent_state->pos()(0) - state_->pos()(0));

        // Set the heading
        desired_state_->quat().set(0, 0, heading); // roll, pitch, heading

        // Match entity's altitude
        desired_state_->pos()(2) = ent_state->pos()(2);
    }

    return true;
}
