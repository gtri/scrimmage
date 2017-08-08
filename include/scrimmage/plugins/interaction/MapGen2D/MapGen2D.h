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

#ifndef MapGen2D_H_
#define MapGen2D_H_
#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/proto/Shape.pb.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

class MapGen2D : public scrimmage::EntityInteraction
{
public:
    MapGen2D();
    bool init(std::map<std::string,std::string> &mission_params,
              std::map<std::string,std::string> &plugin_params);
    bool step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                 double t, double dt);
protected:
    std::shared_ptr<sp::Shape> connect_points(Eigen::Vector3d &p,
                                              Eigen::Vector3d &prev_p);

    double wall_bottom_z_;
    double wall_height_;
    double wall_thickness_;
    double resolution_;
    
private:     
};

#endif
