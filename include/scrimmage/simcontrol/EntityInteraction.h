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

#ifndef INCLUDE_SCRIMMAGE_SIMCONTROL_ENTITYINTERACTION_H_
#define INCLUDE_SCRIMMAGE_SIMCONTROL_ENTITYINTERACTION_H_

#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/entity/EntityPlugin.h>

#include <list>
#include <map>
#include <memory>
#include <string>

namespace scrimmage {

class EntityInteraction : public EntityPlugin {

 public:
    inline virtual bool init(std::map<std::string, std::string> &/*mission_params*/,
                      std::map<std::string, std::string> &/*plugin_params*/)
    { return true;}

    inline virtual bool step_entity_interaction(std::list<EntityPtr> &/*ents*/,
                                                double /*t*/, double /*dt*/)
    { return false; }

    inline virtual bool collision_exists(std::list<EntityPtr> &/*ents*/,
                                  Eigen::Vector3d &/*p*/)
    { return false; }

 protected:
};

typedef std::shared_ptr<EntityInteraction> EntityInteractionPtr;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_SIMCONTROL_ENTITYINTERACTION_H_
