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

#include <scrimmage/plugins/interaction/FlagCaptureInteraction/FlagCaptureInteraction.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
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
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::FlagCaptureInteraction,
                FlagCaptureInteraction_plugin)

namespace scrimmage {
namespace interaction {

FlagCaptureInteraction::FlagCaptureInteraction() {
}

bool FlagCaptureInteraction::init(std::map<std::string, std::string> &mission_params,
                                  std::map<std::string, std::string> &plugin_params) {
    flag_boundary_id_ = sc::get<int>("flag_boundary_id", plugin_params, 2);
    capture_boundary_id_ = sc::get<int>("capture_boundary_id", plugin_params, 1);

    auto callback = [&] (scrimmage::MessagePtr<sp::Shape> msg) {
        if (msg->data.id().id() == flag_boundary_id_) {
            flag_boundary_shape_ = msg->data;
            flag_boundary_ = sci::Boundary::make_boundary(msg->data);
        } else if (msg->data.id().id() == capture_boundary_id_) {
            capture_boundary_shape_ = msg->data;
            capture_boundary_ = sci::Boundary::make_boundary(msg->data);
        }
    };
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", callback);

    flag_taken_pub_ = advertise("GlobalNetwork", "FlagTaken");
    flag_captured_pub_ = advertise("GlobalNetwork", "FlagCaptured");

    return true;
}

bool FlagCaptureInteraction::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                     double t, double dt) {

    if (flag_boundary_ == nullptr || capture_boundary_ == nullptr) {
        return true;
    }

    // If the flag is not currently captured, determine if an entity from the
    // opposing team is within the flag's boundary
    if (!flag_taken_) {
        for (sc::EntityPtr ent : ents) {
            if (ent->id().team_id() != flag_boundary_shape_.id().team_id() &&
                flag_boundary_->contains(ent->state_truth()->pos())) {
                flag_taken_ = true;
                entity_with_flag_ = ent->id();

                auto msg =
                    std::make_shared<sc::Message<sm::FlagTaken>>();
                msg->data.set_entity_id(ent->id().id());
                msg->data.set_entity_team_id(ent->id().team_id());
                msg->data.set_flag_boundary_id(flag_boundary_shape_.id().id());
                msg->data.set_flag_team_id(flag_boundary_shape_.id().team_id());
                flag_taken_pub_->publish(msg);
                // std::cout << "FLAG " << flag_boundary_info_.id.id()
                //           << " TAKEN by : " << ent->id().id() << std::endl;
            }
        }
    } else if (!flag_captured_) {
        auto it = id_to_ent_map_->find(entity_with_flag_.id());
        if (it != id_to_ent_map_->end() &&
            it->second != nullptr && it->second->state_truth() != nullptr) {
            if (capture_boundary_->contains(it->second->state_truth()->pos())) {
                flag_captured_ = true;

                auto msg =
                    std::make_shared<sc::Message<sm::FlagCaptured>>();
                msg->data.set_entity_id(entity_with_flag_.id());
                msg->data.set_entity_team_id(entity_with_flag_.team_id());
                msg->data.set_flag_boundary_id(flag_boundary_shape_.id().id());
                msg->data.set_flag_team_id(flag_boundary_shape_.id().team_id());
                flag_captured_pub_->publish(msg);
                // std::cout << "FLAG " << flag_boundary_info_.id.id()
                // << " captured by : " << entity_with_flag_.id() << std::endl;
            }
        } else {
            flag_taken_ = false;
        }
    }
    return true;
}
} // namespace interaction
} // namespace scrimmage
