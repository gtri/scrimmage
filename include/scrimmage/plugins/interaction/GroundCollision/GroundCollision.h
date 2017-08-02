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

#ifndef GROUND_COLLISION_H_
#define GROUND_COLLISION_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>

namespace sc = scrimmage;

class GroundCollision : public scrimmage::EntityInteraction {
public:
    GroundCollision();

    virtual std::string name()
    { return std::string("GroundCollision"); }
    
    virtual bool init(std::map<std::string,std::string> &mission_params,
                      std::map<std::string,std::string> &plugin_params);

    virtual bool step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                         double t, double dt);

    virtual bool collision_exists(std::list<sc::EntityPtr> &ents,
                                  Eigen::Vector3d &p);
protected:
    double ground_collision_z_;
    sc::PublisherPtr collision_pub_;
private:
};

#endif
