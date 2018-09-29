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
#include <scrimmage/common/Time.h>
#include <scrimmage/common/ParameterServer.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/RTree.h>

#include <scrimmage/plugins/interaction/SimpleCollision/SimpleCollision.h>

#include <limits>
#include <memory>

namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction, scrimmage::interaction::SimpleCollision, SimpleCollision_plugin)

namespace scrimmage {
namespace interaction {

bool SimpleCollision::init(std::map<std::string, std::string> &mission_params,
                           std::map<std::string, std::string> &plugin_params) {
    collision_range_ = get("collision_range", plugin_params, 0.0);

    // If startup_collision_range isn't defined, default to the collision_range
    startup_collision_range_ = get("startup_collision_range",
                                       plugin_params, collision_range_);

    startup_collisions_only_ = get("startup_collisions_only", plugin_params, false);

    enable_team_collisions_ = get<bool>("enable_team_collisions", plugin_params, true);
    enable_non_team_collisions_ = get<bool>("enable_non_team_collisions", plugin_params, true);

    init_alt_deconflict_ = get<bool>("init_alt_deconflict", plugin_params, false);

    // Setup publishers
    team_collision_pub_ = advertise("GlobalNetwork", "TeamCollision");
    non_team_collision_pub_ = advertise("GlobalNetwork", "NonTeamCollision");

    return true;
}


bool SimpleCollision::step_entity_interaction(std::list<EntityPtr> &ents,
                                              double t, double dt) {
    if (startup_collisions_only_) {
        return true;
    }

    // Account for entities "colliding"
    for (EntityPtr ent1 : ents) {
        Eigen::Vector3d p1 = ent1->state()->pos();
        for (EntityPtr ent2 : ents) {
            // ignore distance between itself
            if (ent1->id().id() == ent2->id().id()) continue;

            // ignore collisions that have already occurred this time-step
            if (!ent1->is_alive() || !ent2->is_alive()) continue;

            Eigen::Vector3d p2 = ent2->state()->pos();

            double dist = (p1 - p2).norm();
            if (dist < collision_range_) {
                if (enable_team_collisions_ &&
                    ent1->id().team_id() == ent2->id().team_id()) {

                    ent1->collision();
                    ent2->collision();

                    auto msg = std::make_shared<Message<sm::TeamCollision>>();
                    msg->data.set_entity_id_1(ent1->id().id());
                    msg->data.set_entity_id_2(ent2->id().id());
                    team_collision_pub_->publish(msg);

                } else if (enable_non_team_collisions_ &&
                           ent1->id().team_id() != ent2->id().team_id()) {
                    ent1->collision();
                    ent2->collision();

                    auto msg = std::make_shared<Message<sm::NonTeamCollision>>();
                    msg->data.set_entity_id_1(ent1->id().id());
                    msg->data.set_entity_id_2(ent2->id().id());
                    non_team_collision_pub_->publish(msg);
                }
            }
        }
    }
    return true;
}

bool SimpleCollision::collision_exists(std::list<EntityPtr> &ents,
                                       Eigen::Vector3d &p) {
    if (ents.empty()) {
        return false;
    } else {
        if (init_alt_deconflict_) {
            for (EntityPtr ent : ents) {
                if (std::abs(p(2) - ent->state()->pos()(2)) <=
                    startup_collision_range_) {
                    return true;
                }
            }
        } else {
            if (ents.front()->autonomies().empty()) {
                return false;
            } else {
                std::vector<ID> neighbors;
                RTreePtr rtree = ents.front()->autonomies().front()->rtree();
                rtree->neighbors_in_range(p, neighbors, startup_collision_range_);
                return !neighbors.empty();
            }
        }
    }
    return false;
}
} // namespace interaction
} // namespace scrimmage
