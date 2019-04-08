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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/msgs/Capture.pb.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/interaction/SimpleCapture/SimpleCapture.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <memory>
#include <limits>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
        scrimmage::interaction::SimpleCapture, SimpleCapture_plugin)

namespace scrimmage {
namespace interaction {

SimpleCapture::SimpleCapture(): capture_range_(0),
    enable_team_captures_(true), enable_non_team_captures_(true) {
}
bool SimpleCapture::init(std::map<std::string, std::string> &mission_params,
                         std::map<std::string, std::string> &plugin_params) {
    capture_range_ = sc::get("capture_range", plugin_params, 0.0);

    enable_team_captures_ =
        sc::get<bool>("enable_team_captures", plugin_params, true);
    enable_non_team_captures_ =
        sc::get<bool>("enable_non_team_captures", plugin_params, true);

    // Setup publishers
    team_capture_pub_ = advertise("GlobalNetwork", "TeamCapture");
    non_team_capture_pub_ = advertise("GlobalNetwork", "NonTeamCapture");

    // Setup capture callback
    auto capture_cb = [&] (scrimmage::MessagePtr<sm::CaptureEntity> msg) {
        int source_id = msg->data.source_id();
        int target_id = msg->data.target_id();

        // if target_id is already present in already_captured_,
        // capture_result.second will be false
        auto capture_result = already_captured_.emplace(target_id);

        if (capture_result.second) {
            sc::EntityPtr &src = (*id_to_ent_map_)[source_id];
            sc::EntityPtr &dst = (*id_to_ent_map_)[target_id];
            if ((src->state_truth()->pos() - dst->state_truth()->pos()).norm() <=
                capture_range_) {
                if (enable_team_captures_ &&
                    src->id().team_id() == dst->id().team_id()) {
                    dst->collision();

                    auto msg =
                        std::make_shared<sc::Message<sm::TeamCapture>>();
                    msg->data.set_source_id(src->id().id());
                    msg->data.set_target_id(dst->id().id());
                    team_capture_pub_->publish(msg);

                } else if (enable_non_team_captures_ &&
                           src->id().team_id() != dst->id().team_id()) {
                    dst->collision();

                    auto msg =
                        std::make_shared<sc::Message<sm::NonTeamCapture>>();
                    msg->data.set_source_id(src->id().id());
                    msg->data.set_target_id(dst->id().id());
                    non_team_capture_pub_->publish(msg);
                }
            }
        }
    };
    subscribe<sm::CaptureEntity>("GlobalNetwork", "CaptureEntity", capture_cb);
    return true;
}

bool SimpleCapture::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                            double t, double dt) {
    return true;
}
}  // namespace interaction
}  // namespace scrimmage
