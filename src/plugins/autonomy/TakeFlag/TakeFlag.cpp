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
    pub_wp_list_ = advertise("LocalNetwork", "WaypointList");

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
}

bool TakeFlag::step_autonomy(double t, double dt) {

    if (!has_flag_) {
        // If we don't have the flag yet, head towards it!
        auto it = boundaries_.find(flag_boundary_id_);
        if (it != boundaries_.end()) {
            publish_waypoint(std::get<1>(it->second)->center());
        }
    } else {
        // Once we have the flag, head towards the return boundary
        auto it = boundaries_.find(capture_boundary_id_);
        if (it != boundaries_.end()) {
            publish_waypoint(std::get<1>(it->second)->center());
        }
    }
    return true;
}

void TakeFlag::publish_waypoint(const Eigen::Vector3d &point) {
    auto path_msg = std::make_shared<Message<WaypointList>>();
    path_msg->data.set_mode(WaypointList::WaypointMode::follow_once);
    path_msg->data.set_cycles(1);

    double lat, lon, alt;
    parent_->projection()->Reverse(point(0), point(1), point(2),
                                   lat, lon, alt);

    Waypoint wp(lat, lon, alt);
    wp.set_time(0);
    scrimmage::Quaternion quat(0, 0, 0);
    wp.set_quat(quat);
    wp.set_position_tolerance(80);
    path_msg->data.waypoints().push_back(wp);

    pub_wp_list_->publish(path_msg);
}
} // namespace autonomy
} // namespace scrimmage
