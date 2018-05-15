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
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/sensor/SimpleCamera/SimpleCamera.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/Hash.h>

#include <vector>
#include <unordered_set>

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::SimpleCamera, SimpleCamera_plugin)

namespace scrimmage {
namespace sensor {

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

void SimpleCamera::init(std::map<std::string, std::string> &params) {
    range_ = std::stod(params.at("range"));
    fov_az_ = sc::Angles::deg2rad(std::stod(params.at("fov_az")));
    fov_el_ = sc::Angles::deg2rad(std::stod(params.at("fov_el")));
    draw_cone_ = sc::str2bool(params.at("draw_cone"));
    return;
}

scrimmage::MessageBasePtr SimpleCamera::sensor_msg(double t) {

    int my_id = parent_->id().id();
    auto msg = std::make_shared<sc::Message<std::unordered_set<sc::ID>>>();

    std::vector<sc::ID> neigh;
    sc::ContactMapPtr c = parent_->contacts();
    sc::StatePtr s = (*c)[my_id].state();

    parent_->rtree()->neighbors_in_range(s->pos(), neigh, range_, my_id);

    std::copy_if(neigh.begin(), neigh.end(), std::inserter(msg->data, msg->data.end()),
        [&](sc::ID &id) {return s->InFieldOfView(*c->at(id.id()).state(), fov_az_, fov_el_);});

    if (draw_cone_ && !parent_->autonomies().empty()) {
        sc::Quaternion &q = s->quat();
        double pitch = q.pitch();
        double yaw = q.yaw();
        Eigen::Vector3d orient(cos(yaw) * cos(pitch),
                               sin(yaw) * cos(pitch),
                               -sin(pitch));

        parent_->autonomies().front()->shapes().clear();

        auto cone = std::make_shared<scrimmage_proto::Shape>();
        cone->set_opacity(0.2);
        cone->set_ttl(1);

        sc::set(cone->mutable_cone()->mutable_apex(), s->pos());
        sc::set(cone->mutable_cone()->mutable_direction(), orient);
        cone->mutable_cone()->set_height(range_);
        cone->mutable_cone()->set_base_radius(range_ * sin(fov_az_ / 2.0));
        parent_->autonomies().front()->shapes().push_back(cone);
    }

    return msg;
}
} // namespace sensor
} // namespace scrimmage
