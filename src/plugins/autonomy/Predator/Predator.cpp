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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/msgs/Capture.pb.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/plugins/autonomy/Predator/Predator.h>

#include <limits>

namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::Predator, Predator_plugin)

namespace scrimmage {
namespace autonomy {

void Predator::init(std::map<std::string, std::string> &params) {
    max_speed_ = get<double>("max_speed", params, 21);
    capture_range_ = get<double>("capture_range", params, 5);
    prey_team_id_ = get<int>("prey_team_id", params, 1);

    allow_prey_switching_ = get<bool>("allow_prey_switching", params, false);

    capture_ent_pub_ = advertise("GlobalNetwork", "CaptureEntity");

    follow_id_ = -1;

    speed_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::Out);
    turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);

    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
}

bool Predator::step_autonomy(double t, double dt) {
    if (contacts_->count(follow_id_) == 0) {
        follow_id_ = -1;
    }

    if (follow_id_ < 0 || allow_prey_switching_) {
        // Find nearest entity on other team. Loop through each contact, calculate
        // distance to entity, save the ID of the entity that is closest.
        double min_dist = std::numeric_limits<double>::infinity();
        for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
            // Skip if this contact isn't on team 1 (prey)
            // if (it->second.id().team_id() == parent_->id().team_id()) continue;
            if (it->second.id().team_id() != prey_team_id_) continue;

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
            auto msg = std::make_shared<Message<sm::CaptureEntity>>();
            msg->data.set_source_id(parent_->id().id());
            msg->data.set_target_id(it->second.id().id());
            capture_ent_pub_->publish(msg);
        }
    }

    // Head toward entity on other team
    if (contacts_->count(follow_id_) > 0) {
        // Get a reference to the entity's state.
        StatePtr ent_state = contacts_->at(follow_id_).state();

        // Calculate max velocity towards target entity:
        Eigen::Vector3d v = (ent_state->pos() - state_->pos()).normalized() * max_speed_;

        // Convert to spherical coordinates:
        double desired_heading = atan2(v(1), v(0));
        double desired_pitch = atan2(v(2), v.head<2>().norm());

        vars_.output(speed_idx_, max_speed_);
        vars_.output(turn_rate_idx_, Angles::angle_pi(desired_heading - state_->quat().yaw()));
        vars_.output(pitch_rate_idx_, Angles::angle_pi(desired_pitch + state_->quat().pitch()));

        vars_.output(desired_heading_idx_, desired_heading);
        vars_.output(desired_speed_idx_, max_speed_);
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
