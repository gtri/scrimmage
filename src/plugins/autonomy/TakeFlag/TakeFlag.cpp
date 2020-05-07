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

#include <scrimmage/plugins/autonomy/TakeFlag/TakeFlag.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>

#include <scrimmage/common/Waypoint.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointList.h>

#include <scrimmage/msgs/Capture.pb.h>

#include <iostream>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sm = scrimmage_msgs;
namespace sci = scrimmage::interaction;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::TakeFlag,
                TakeFlag_plugin)

namespace scrimmage {
namespace autonomy {

TakeFlag::TakeFlag() {
}

void TakeFlag::init(std::map<std::string, std::string> &params) {
    flag_boundary_id_ = get<int>("flag_boundary_id", params, 1);
    capture_boundary_id_ = get<int>("capture_boundary_id", params, 1);

    auto callback = [&] (scrimmage::MessagePtr<sp::Shape> msg) {
        std::shared_ptr<sci::BoundaryBase> boundary = sci::Boundary::make_boundary(msg->data);
        boundaries_[msg->data.id().id()] = std::make_pair(msg->data, boundary);
    };
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", callback);

    auto flag_taken_cb = [&] (scrimmage::MessagePtr<sm::FlagTaken> msg) {
        if (msg->data.entity_id() == parent_->id().id() &&
            msg->data.flag_boundary_id() == flag_boundary_id_) {
            has_flag_ = true;
        }
    };
    subscribe<sm::FlagTaken>("GlobalNetwork", "FlagTaken", flag_taken_cb);

    output_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
    output_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
    output_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);
}

bool TakeFlag::step_autonomy(double t, double dt) {

    Eigen::Vector3d desired_point = state_->pos();
    if (!has_flag_) {
        // If we don't have the flag yet, head towards it!
        auto it = boundaries_.find(flag_boundary_id_);
        if (it != boundaries_.end()) {
            desired_point = std::get<1>(it->second)->center();
        }
    } else {
        // Once we have the flag, head towards the return boundary
        auto it = boundaries_.find(capture_boundary_id_);
        if (it != boundaries_.end()) {
            desired_point = std::get<1>(it->second)->center();
        }
    }

    // Convert the desired point into a desired vector, relative to our own
    // position.
    Eigen::Vector3d desired_velocity = desired_point - state_->pos();

    vars_.output(output_vel_x_idx_, desired_velocity(0));
    vars_.output(output_vel_y_idx_, desired_velocity(1));
    vars_.output(output_vel_z_idx_, desired_velocity(2));

    return true;
}

} // namespace autonomy
} // namespace scrimmage
