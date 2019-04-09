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
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/motion/MotionModel.h>

#include <scrimmage/plugins/interaction/GroundCollision/GroundCollision.h>

#include <memory>

#include <GeographicLib/LocalCartesian.hpp>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction, scrimmage::interaction::GroundCollision, GroundCollision_plugin)

namespace scrimmage {
namespace interaction {

GroundCollision::GroundCollision() : ground_collision_z_(0.0),
                                     remove_on_collision_(true),
                                     enable_startup_collisions_(true) {
}

bool GroundCollision::init(std::map<std::string, std::string> &mission_params,
                           std::map<std::string, std::string> &plugin_params) {

    remove_on_collision_ = sc::get<bool>("remove_on_collision", plugin_params, true);
    enable_startup_collisions_ = sc::get<bool>("enable_startup_collisions", plugin_params, true);

    // Determine ground collision z-value. If ground_collision_altitude is
    // defined, it has priority.
    ground_collision_z_ = sc::get("ground_collision_z", plugin_params, 0.0);
    if (plugin_params.count("ground_collision_altitude") > 0 && parent_->mp()) {
        double x, y, z, alt;
        alt = sc::get("ground_collision_altitude", plugin_params, 0.0);
        parent_->projection()->Forward(parent_->mp()->latitude_origin(),
                                       parent_->mp()->longitude_origin(), alt,
                                       x, y, z);
        ground_collision_z_ = z;
    }

    collision_pub_ = advertise("GlobalNetwork", "GroundCollision");

    team_ = plugin_params.at("team");
    return true;
}

bool GroundCollision::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                              double t, double dt) {
    // Account for entities colliding with
    for (sc::EntityPtr ent : ents) {
        if (team_ != "all" && std::stoi(team_) != ent->id().team_id()) {
            continue;
        }

        if (ent->is_alive() && ent->state_truth()->pos()(2) <= ground_collision_z_) {
            if (remove_on_collision_) {
                ent->collision();
            } else {
                // Apply a normal force to motion model in opposite direction
                // of gravity.
                double force = ent->motion()->mass() * ent->motion()->gravity_magnitude();
                ent->motion()->set_external_force(Eigen::Vector3d(0, 0, force));
                // StatePtr &s = ent->state_truth();
                // s->pos()(2) = ground_collision_z_;
                // s->vel() << 0, 0, 0;
                // ent->motion()->teleport(s);
            }

            auto msg = std::make_shared<sc::Message<sm::GroundCollision>>();
            msg->data.set_entity_id(ent->id().id());
            collision_pub_->publish(msg);
        }
    }
    return true;
}

bool GroundCollision::collision_exists(std::list<sc::EntityPtr> &ents,
                                       Eigen::Vector3d &p) {

    if (!enable_startup_collisions_) {
        return false;
    }

    if (p(2) <= ground_collision_z_) {
        return true;
    }
    return false;
}
} // namespace interaction
} // namespace scrimmage
