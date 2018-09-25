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

#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointGenerator.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/Waypoint.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointList.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/common/VariableIO.h>

#include <iostream>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::WaypointGenerator,
                WaypointGenerator_plugin)

namespace scrimmage {
namespace autonomy {

WaypointGenerator::WaypointGenerator() {
}

void WaypointGenerator::init(std::map<std::string, std::string> &params) {
    waypoint_color_ = sc::str2container<std::vector<int>>(
        sc::get<std::string>("waypoint_color", params, "255,0,0"), ",");

    show_waypoints_ = sc::get<bool>("show_waypoints", params, false);

    std::string waypoints = sc::get<std::string>("waypoints", params, "");

    std::vector<std::vector<std::string>> vecs;
    if (!sc::get_vec_of_vecs(waypoints, vecs)) {
        cout << "Failed to parse waypoints:" << waypoints << endl;
    } else {
        for (std::vector<std::string> vec : vecs) {
            if (vec.size() != 10) {
                cout << "Invalid waypoint: " << waypoints << endl;
                continue;
            }

            double lat = std::stod(vec[2]);
            double lon = std::stod(vec[3]);
            double alt = std::stod(vec[4]);

            std::string type = vec[0];
            std::transform(type.begin(), type.end(), type.begin(), ::toupper);
            if (type != "XYZ" && type != "GPS") {
                cout << "Invalid waypoint type: " << type << endl;
            } else if (type == "XYZ") {
                // Convert to XYZ point to GPS
                parent_->projection()->Reverse(lat, lon, alt,
                                               lat, lon, alt);
            }

            Waypoint wp(lat, lon, alt);
            wp.set_time(std::stod(vec[1]));
            wp.quat().set(sc::Angles::deg2rad(std::stod(vec[5])),
                          sc::Angles::deg2rad(std::stod(vec[6])),
                          sc::Angles::deg2rad(std::stod(vec[7])));

            wp.set_position_tolerance(std::stod(vec[8]));
            wp.set_quat_tolerance(std::stod(vec[9]));

            wp_list_.waypoints().push_back(wp);
        }
    }

    std::string mode = sc::get<std::string>("mode", params, "follow_once");
    if (mode == "follow_once") {
        wp_list_.set_mode(WaypointList::WaypointMode::follow_once);
    } else if (mode == "back_and_forth") {
        wp_list_.set_mode(WaypointList::WaypointMode::back_and_forth);
    } else if (mode == "loiter") {
        wp_list_.set_mode(WaypointList::WaypointMode::loiter);
    } else if (mode == "racetrack") {
        wp_list_.set_mode(WaypointList::WaypointMode::racetrack);
    } else {
        cout << "WaypointGenerator: Invalid mode. Defaulting to follow_once" << endl;
        wp_list_.set_mode(WaypointList::WaypointMode::follow_once);
    }

    std::string network_name = sc::get<std::string>("network_name", params, "GlobalNetwork");
    std::string topic_name = sc::get<std::string>("topic_name", params, "WaypointList");
    waypoint_list_pub_ = advertise(network_name, topic_name);

    position_x_idx_ = vars_.declare("position_x", VariableIO::Direction::Out);
    position_y_idx_ = vars_.declare("position_y", VariableIO::Direction::Out);
    position_z_idx_ = vars_.declare("position_z", VariableIO::Direction::Out);
}

bool WaypointGenerator::step_autonomy(double t, double dt) {
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
        if (show_waypoints_) {
            draw_waypoints(msg->data);
        }
        waypoint_list_pub_->publish(msg);
    }

    return true;
}

void WaypointGenerator::draw_waypoints(WaypointList &wp_list) {
    for (Waypoint wp : wp_list.waypoints()) {
        double x, y, z;
        parent_->projection()->Forward(wp.latitude(), wp.longitude(), wp.altitude(),
                                       x, y, z);
        vars_.output(position_x_idx_, x);
        vars_.output(position_y_idx_, y);
        vars_.output(position_z_idx_, z);

        auto sphere = std::make_shared<scrimmage_proto::Shape>();
        sphere->set_opacity(0.25);
        sphere->set_persistent(true);
        sc::set(sphere->mutable_color(), waypoint_color_[0], waypoint_color_[1],
                waypoint_color_[2]);

        sphere->mutable_sphere()->set_radius(wp.position_tolerance());
        sc::set(sphere->mutable_sphere()->mutable_center(), x, y, z);
        draw_shape(sphere);
    }
}
} // namespace autonomy
} // namespace scrimmage
