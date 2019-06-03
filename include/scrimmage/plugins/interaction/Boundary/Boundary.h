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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_BOUNDARY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_BOUNDARY_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/proto/Shape.pb.h>

#include <map>
#include <string>
#include <list>
#include <memory>

namespace sc = scrimmage;

namespace scrimmage {
namespace interaction {

class Boundary : public scrimmage::EntityInteraction {
 public:
    Boundary();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                 double t, double dt) override;

    static std::shared_ptr<BoundaryBase> make_boundary(const scrimmage_proto::Shape &shape);

 protected:
    sc::PublisherPtr pub_boundary_;
    bool boundary_published_ = false;
    std::shared_ptr<scrimmage_proto::Shape> boundary_shape_;
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_BOUNDARY_H_
