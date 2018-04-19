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

#include <scrimmage/plugins/interaction/FlagCaptureInteraction/FlagCaptureInteraction.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Capture.pb.h>

#include <scrimmage/plugins/interaction/Boundary/BoundaryInfo.h>
#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>
#include <scrimmage/plugins/interaction/Boundary/Sphere.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::FlagCaptureInteraction,
                FlagCaptureInteraction_plugin)

namespace scrimmage {
namespace interaction {

FlagCaptureInteraction::FlagCaptureInteraction(): flag_boundary_id_(2),
                                                  capture_boundary_id_(1),
                                                  flag_taken_(false) {
}

bool FlagCaptureInteraction::init(std::map<std::string, std::string> &mission_params,
                                  std::map<std::string, std::string> &plugin_params) {
    flag_boundary_id_ = sc::get<int>("flag_boundary_id", plugin_params, 2);
    capture_boundary_id_ = sc::get<int>("capture_boundary_id", plugin_params, 1);

    auto callback = [&] (scrimmage::MessagePtr<sci::BoundaryInfo> msg) {
        if (msg->data.id.id() == flag_boundary_id_) {
            flag_boundary_info_ = msg->data;
            if (msg->data.type == sci::BoundaryInfo::Type::Cuboid) {
                std::shared_ptr<sci::Cuboid> cuboid = std::make_shared<sci::Cuboid>();
                cuboid->set_points(msg->data.points);
                flag_boundary_ = cuboid;
            } else if (msg->data.type == sci::BoundaryInfo::Type::Sphere) {
                std::shared_ptr<sci::Sphere> sphere = std::make_shared<sci::Sphere>();
                sphere->set_radius(msg->data.radius);
                sphere->set_center(msg->data.center);
                flag_boundary_ = sphere;
            } else {
                std::cout << "Ignoring boundary: " << msg->data.name << std::endl;
            }
        } else if (msg->data.id.id() == capture_boundary_id_) {
            capture_boundary_info_ = msg->data;
            if (msg->data.type == sci::BoundaryInfo::Type::Cuboid) {
                std::shared_ptr<sci::Cuboid> cuboid = std::make_shared<sci::Cuboid>();
                cuboid->set_points(msg->data.points);
                capture_boundary_ = cuboid;
            } else if (msg->data.type == sci::BoundaryInfo::Type::Sphere) {
                std::shared_ptr<sci::Sphere> sphere = std::make_shared<sci::Sphere>();
                sphere->set_radius(msg->data.radius);
                sphere->set_center(msg->data.center);
                capture_boundary_ = sphere;
            } else {
                std::cout << "Ignoring boundary: " << msg->data.name << std::endl;
            }
        }
    };
    subscribe<sci::BoundaryInfo>("GlobalNetwork", "Boundary", callback);

    flag_taken_pub_ = advertise("GlobalNetwork", "FlagTaken");

    return true;
}


bool FlagCaptureInteraction::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                     double t, double dt) {

    if (flag_boundary_ == nullptr || capture_boundary_ == nullptr) {
        return true;
    }

    // If the flag is not currently captured, determine if an entity from the
    // opposing team is within the flag's boundary
    if (!flag_taken_) {
        for (sc::EntityPtr ent : ents) {
            if (ent->id().team_id() != flag_boundary_info_.id.team_id() &&
                flag_boundary_->contains(ent->state()->pos())) {
                flag_taken_ = true;

                auto msg =
                    std::make_shared<sc::Message<sm::FlagTaken>>();
                msg->data.set_entity_id(ent->id().id());
                msg->data.set_entity_team_id(ent->id().team_id());
                msg->data.set_flag_boundary_id(flag_boundary_info_.id.id());
                msg->data.set_flag_team_id(flag_boundary_info_.id.team_id());
                flag_taken_pub_->publish(msg);
                // std::cout << "FLAG " << flag_boundary_info_.id.id()
                //           << " TAKEN by : " << ent->id().id() << std::endl;
            }
        }
    }
    return true;
}
} // namespace interaction
} // namespace scrimmage
