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

#include <scrimmage/plugins/autonomy/GoToWaypoint/GoToWaypoint.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/pubsub/Publisher.h>
#include <iostream>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::GoToWaypoint,
                GoToWaypoint_plugin)

namespace scrimmage {
namespace autonomy {

void GoToWaypoint::init(std::map<std::string, std::string> &params) {
    waypoint_status_ = str2container(params["waypoint"], " ", waypoint_);

    waypoint_list_pub_ = advertise("LocalNetwork", "WaypointList");
}

bool GoToWaypoint::step_autonomy(double t, double dt) {
    if (waypoint_status_) {

        sc::State ns = *(parent_->state());

        double lat = (waypoint_[2] == "X") ? ns.pos()(0): stod(waypoint_[2]);
        double lon = (waypoint_[3] == "Y") ? ns.pos()(1): stod(waypoint_[3]);
        double alt = (waypoint_[4] == "Z") ? ns.pos()(2): stod(waypoint_[4]);

        std::string type = waypoint_[0];
        if (type != "XYZ" && type != "GPS") {
            std::cout<< "Invalid waypoint type: " << type << std::endl;
        } else if (type == "XYZ") {
            parent_->projection()->Reverse(lat, lon, alt,
                                               lat, lon, alt);
        }

        Waypoint wp(lat, lon, alt);
        wp.set_time(std::stod(waypoint_[1]));
        scrimmage::Quaternion quat(sc::Angles::deg2rad(std::stod(waypoint_[5])),
                                   sc::Angles::deg2rad(std::stod(waypoint_[6])),
                                   sc::Angles::deg2rad(std::stod(waypoint_[7])));
        wp.set_quat(quat);
        wp.set_position_tolerance(std::stod(waypoint_[8]));
        wp.set_quat_tolerance(std::stod(waypoint_[9]));

        wp_list_.waypoints().push_back(wp);

        publish_waypoint(t);
    }

    return true;
}

void GoToWaypoint::publish_waypoint(double t) {
    auto msg = std::make_shared<sc::Message<WaypointList>>();

    std::list<Waypoint>::iterator it = wp_list_.waypoints().begin();
    while (it != wp_list_.waypoints().end()) {
        if (t >= it->time()) {
            msg->data.set_mode(wp_list_.mode());
            msg->data.waypoints().push_back(*it);

            // Erase this waypoint from the main list
            std::list<Waypoint>::iterator it_del = it;
            it = wp_list_.waypoints().erase(it_del);
        } else {
            ++it;
        }
    }

    // Publish the waypoint list message if it has waypoints
    if (msg->data.waypoints().size() > 0) {
        waypoint_list_pub_->publish(msg);
    }
}

} // namespace autonomy
} // namespace scrimmage
