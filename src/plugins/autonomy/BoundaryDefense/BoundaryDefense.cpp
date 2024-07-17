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
#include <scrimmage/common/Time.h>
#include <scrimmage/common/Waypoint.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/BoundaryDefense/BoundaryDefense.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointList.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <iostream>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sci = scrimmage::interaction;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::BoundaryDefense, BoundaryDefense_plugin)

namespace scrimmage {
namespace autonomy {

BoundaryDefense::BoundaryDefense() {}

void BoundaryDefense::init(std::map<std::string, std::string> &params) {
    boundary_id_ = sc::get<int>("boundary_id", params, boundary_id_);

    auto callback = [&](scrimmage::MessagePtr<sp::Shape> msg) {
        std::shared_ptr<sci::BoundaryBase> boundary = sci::Boundary::make_boundary(msg->data);
        boundaries_[msg->data.id().id()] = std::make_pair(msg->data, boundary);
    };
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", callback);

    output_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
    output_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
    output_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);
}

bool BoundaryDefense::step_autonomy(double t, double dt) {
    // Get the active boundary to defend
    auto it = boundaries_.find(boundary_id_);
    if (it == boundaries_.end()) {
        return true;
    }

    // Find the closest entity within the boundary that is not part of the
    // boundary's team.
    double min_dist = std::numeric_limits<double>::infinity();
    sc::StatePtr closest = nullptr;
    for (auto &kv : *contacts_) {
        sc::Contact &cnt = kv.second;

        // Ignore same team
        if (cnt.id().team_id() == std::get<0>(it->second).id().team_id()) {
            continue;
        }

        // Is the entity within the boundary?
        if (std::get<1>(it->second)->contains(cnt.state()->pos())) {
            if (closest == nullptr || (state_->pos() - closest->pos()).norm() < min_dist) {
                closest = cnt.state();
                min_dist = (state_->pos() - closest->pos()).norm();
            }
        }
    }

    Eigen::Vector3d desired_point;
    if (closest != nullptr) {
        // Chase the entity
        desired_point = closest->pos() + closest->vel() * time_->dt();
    } else {
        // If there are no entities to chase, move to center of boundary that
        // we are trying to defend
        desired_point = std::get<1>(it->second)->center();
    }

    // Convert the desired point into a desired vector, relative to our own
    // position.
    Eigen::Vector3d desired_velocity = desired_point - state_->pos();

    vars_.output(output_vel_x_idx_, desired_velocity(0));
    vars_.output(output_vel_y_idx_, desired_velocity(1));
    vars_.output(output_vel_z_idx_, desired_velocity(2));

    return true;
}
}  // namespace autonomy
}  // namespace scrimmage
