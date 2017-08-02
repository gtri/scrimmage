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
#include "SimpleCamera.h"
#include <scrimmage/common/ID.h>
#include <vector>
#include <scrimmage/common/RTree.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/parse/ParseUtils.h>

REGISTER_PLUGIN(scrimmage::Sensor, SimpleCamera, SimpleCamera_plugin)

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

void SimpleCamera::init(std::map<std::string,std::string> &params) {
    range_ = std::stod(params.at("range"));
    fov_azimuth_ = sc::Angles::deg2rad(std::stod(params.at("fov_azimuth")));
    fov_elevation_ = sc::Angles::deg2rad(std::stod(params.at("fov_elevation")));
    draw_cone_ = sc::str2bool(params.at("draw_cone"));
    return;
}

bool SimpleCamera::sense(double t, double dt) {

    int my_id = parent_->id().id();

    std::vector<sc::ID> neigh;
    sc::StatePtr &state = contacts_->at(my_id).state();

    rtree_->neighbors_in_range(state->pos(), neigh, range_, my_id);

    sensed_contacts_.clear();
    for (sc::ID &id : neigh) {
        sc::State &other_state = *contacts_->at(id.id()).state();
        if (state->InFieldOfView(other_state, fov_azimuth_, fov_elevation_)) {
            sensed_contacts_[id.id()] = contacts_->at(id.id());
        }
    }

    if (draw_cone_ && !parent_->autonomies().empty()) {
        sc::Quaternion &q = state->quat();
        double pitch = q.pitch();
        double yaw = q.yaw();
        Eigen::Vector3d orient(cos(yaw) * cos(pitch),
                               sin(yaw) * cos(pitch),
                               -sin(pitch));

        parent_->autonomies().front()->shapes().clear();
        std::shared_ptr<scrimmage_proto::Shape> cone(new scrimmage_proto::Shape());
        sc::set(cone->mutable_apex(), state->pos());
        cone->set_opacity(0.2);
        sc::set(cone->mutable_direction(), orient);
        cone->set_height(range_);
        cone->set_base_radius(range_ * sin(fov_azimuth_ / 2.0));
        cone->set_type(scrimmage_proto::Shape::Cone);
        cone->set_ttl(1);
        parent_->autonomies().front()->shapes().push_back(cone);
    } 

    return true;
}
