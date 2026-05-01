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
 * @author Ethan M Boos <ethan.m.boos@gtri.gatech.edu>
 * @date 01 May 2026
 * @version 0.1.0
 * @brief Drop-in O(n) SimpleCollision replacement.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_SPATIALCOLLISION_SPATIALCOLLISION_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_SPATIALCOLLISION_SPATIALCOLLISION_H_

#include <list>
#include <map>
#include <string>

#include "scrimmage/entity/Entity.h"
#include "scrimmage/simcontrol/EntityInteraction.h"

namespace scrimmage {
namespace interaction {

class SpatialCollision : public scrimmage::EntityInteraction {
 public:
    bool init(
        std::map<std::string, std::string>& mission_params,
        std::map<std::string, std::string>& plugin_params) override;

    bool step_entity_interaction(std::list<scrimmage::EntityPtr>& ents, double t, double dt)
        override;

    bool collision_exists(std::list<scrimmage::EntityPtr>& ents, Eigen::Vector3d& p) override;

 protected:
    bool handle_collision(scrimmage::EntityPtr& ent1, scrimmage::EntityPtr& ent2);

    double collision_range_ = 0.0;
    double startup_collision_range_ = 0.0;
    bool startup_collisions_only_ = false;
    bool enable_team_collisions_ = true;
    bool enable_non_team_collisions_ = true;
    bool init_alt_deconflict_ = false;

    scrimmage::PublisherPtr team_collision_pub_;
    scrimmage::PublisherPtr non_team_collision_pub_;
};

}  // namespace interaction
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_SPATIALCOLLISION_SPATIALCOLLISION_H_