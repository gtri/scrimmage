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

#include <limits>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <memory>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/msgs/Collision.pb.h>

#include <GeographicLib/LocalCartesian.hpp>

#include <scrimmage/plugins/interaction/GroundCollision/GroundCollision.h>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction, GroundCollision,
                GroundCollision_plugin)

GroundCollision::GroundCollision()
{
}

bool GroundCollision::init(std::map<std::string,std::string> &mission_params,
                           std::map<std::string,std::string> &plugin_params)
{
    // Determine ground collision z-value. If ground_collision_altitude is
    // defined, it has priority.
    ground_collision_z_ = sc::get("ground_collision_z", plugin_params, 0.0);
    if (plugin_params.count("ground_collision_altitude") > 0 && mp_) {
        double x,y,z,alt;
        alt = sc::get("ground_collision_altitude", plugin_params, 0.0);
        proj_->Forward(mp_->latitude_origin(), mp_->longitude_origin(), alt, x, y, z);
        ground_collision_z_ = z;
    }

    collision_pub_ = create_publisher("GroundCollision");

    return true;
}


bool GroundCollision::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                              double t, double dt)
{
    // Account for entities colliding with
    for (sc::EntityPtr ent : ents) {
        if (ent->state()->pos()(2) < ground_collision_z_) {
            ent->collision();

            auto msg = std::make_shared<sc::Message<sm::GroundCollision>>();
            msg->data.set_entity_id(ent->id().id());
            publish_immediate(t, collision_pub_, msg);
        }
    }
    return true;
}

bool GroundCollision::collision_exists(std::list<sc::EntityPtr> &ents,
                                       Eigen::Vector3d &p)
{
    if (p(2) < ground_collision_z_) {
        return true;
    }
    return false;
}
