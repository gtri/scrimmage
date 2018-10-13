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

#include <scrimmage/plugins/autonomy/WaypointDispatcher/WaypointDispatcher.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::WaypointDispatcher,
                WaypointDispatcher_plugin)

namespace scrimmage {
namespace autonomy {

Eigen::Vector3d WaypointDispatcher::lla_to_xyz(const Waypoint &wpt) {
    Eigen::Vector3d xyz;
    parent_->projection()->Forward(
        wpt.latitude(), wpt.longitude(), wpt.altitude(), xyz(0), xyz(1), xyz(2));
    return xyz;
}

void WaypointDispatcher::init(std::map<std::string, std::string> &params) {
    lead_distance_ = sc::get<double>("lead_distance", params, lead_distance_);
    filter_dist_ = sc::get<double>("filter_dist", params, filter_dist_);
    show_shapes_ = sc::get<bool>("show_shapes", params, show_shapes_);

    std::string waypointlist_network = get<std::string>(
        "waypointlist_network", params, "GlobalNetwork");
    std::string waypoint_network = sc::get<std::string>(
        "waypoint_network", params, "LocalNetwork");

    wp_pub_ = advertise(waypoint_network, "Waypoint");
    wp_pub_status_ = advertise(waypoint_network, "WaypointStatus");

    auto wp_list_cb = [&] (scrimmage::MessagePtr<WaypointList> msg) {
        // Received a new waypoint list, reset
        wp_list_ = msg->data;
        wp_it_ = wp_list_.waypoints().begin();
        prev_wp_it_ = wp_it_;
        cycles_ = 0;
        returning_stage_ = false;

        // Filter the waypoint list to ensure that two waypoints in succession
        // are not equal, otherwise, the line following will produce NaN
        auto is_close = [&](auto &wpt1, auto &wpt2) {
            return (this->lla_to_xyz(wpt1) - this->lla_to_xyz(wpt2)).norm() < filter_dist_;
        };
        wp_list_.waypoints().unique(is_close);
    };
    subscribe<WaypointList>(waypointlist_network, "WaypointList", wp_list_cb);
}

bool WaypointDispatcher::step_autonomy(double t, double dt) {
    if (wp_list_.waypoints().size() == 0) {
        return true;
    }

    Waypoint curr_wp_lla = *wp_it_;

    Eigen::Vector3d wp;
    parent_->projection()->Forward(curr_wp_lla.latitude(),
                                   curr_wp_lla.longitude(),
                                   curr_wp_lla.altitude(), wp(0), wp(1), wp(2));

    Eigen::Vector3d prev_wp;
    if (prev_wp_it_ == wp_it_) {
        prev_wp = state_->pos();
    } else {
        Waypoint prev_wp_lla = *prev_wp_it_;
        parent_->projection()->Forward(prev_wp_lla.latitude(), prev_wp_lla.longitude(),
                                       prev_wp_lla.altitude(), prev_wp(0),
                                       prev_wp(1), prev_wp(2));
    }

    Eigen::Vector3d wp_line = wp - prev_wp;
    Eigen::Vector3d wp_to_own_line = state_->pos() - prev_wp;

    // Project wp_to_own_line onto line between waypoints (wp_line)
    double dot_prod = wp_line.dot(wp_to_own_line);
    Eigen::Vector3d proj =  dot_prod / pow(wp_line.norm(), 1) *
        wp_line.normalized();

    Eigen::Vector3d track_point = prev_wp + proj + wp_line.normalized() * lead_distance_;

    if (dot_prod < 0) {
        track_point = prev_wp + wp_line.normalized() * (lead_distance_);
    }

    if ((track_point - prev_wp).norm() >= (wp - prev_wp).norm()) {
        track_point = wp;
    }

    auto draw_sphere = [&](auto &sphere, auto &pt, double r, double g, double b) {
        sphere->set_persistent(true);
        sc::set(sphere->mutable_color(), r, g, b);
        sphere->set_opacity(1.0);
        sphere->mutable_sphere()->set_radius(30);
        sc::set(sphere->mutable_sphere()->mutable_center(), pt);
        this->draw_shape(sphere);
    };

    if (show_shapes_) {
        draw_sphere(track_sphere_shape_, track_point, 0, 0, 0);
        draw_sphere(curr_wp_sphere_shape_, wp, 255, 0, 0);
        draw_sphere(prev_wp_sphere_shape_, prev_wp, 0, 0, 255);
    }

    // convert track_point to lat/lon and Publish track_point!
    double lat, lon, alt;
    parent_->projection()->Reverse(track_point(0), track_point(1),
                                   track_point(2), lat, lon, alt);

    auto wp_msg = std::make_shared<sc::Message<Waypoint>>();
    wp_msg->data = Waypoint(lat, lon, alt);
    wp_msg->data.set_id(curr_wp_lla.id());
    wp_pub_->publish(wp_msg);

    // check if within waypoint tolerance
    if ((state_->pos() - wp).norm() <= curr_wp_lla.position_tolerance()) {

        bool done = false;
        switch (wp_list_.mode()) {
        case WaypointList::WaypointMode::follow_once :
            prev_wp_it_ = wp_it_;
            ++wp_it_;

            if (wp_it_ == wp_list_.waypoints().end()) {
                wp_it_ = prev_wp_it_;
                done = true;
            }
            break;

        case WaypointList::WaypointMode::back_and_forth :
            prev_wp_it_ = wp_it_;
            wp_it_ = returning_stage_ ? --wp_it_ : ++wp_it_;

            if (wp_it_ == wp_list_.waypoints().begin()) {
                returning_stage_ = false;
            } else if (wp_it_ == wp_list_.waypoints().end()) {
                wp_it_ = prev_wp_it_;
                returning_stage_ = true;
            }
            break;

        case WaypointList::WaypointMode::loiter :
            // TODO : Not implemented yet
            break;

        case WaypointList::WaypointMode::racetrack :
            // Continuous repeat of waypoints
            prev_wp_it_ = wp_it_;
            ++wp_it_;

            if (wp_it_ == wp_list_.waypoints().end()) {
                wp_it_ = wp_list_.waypoints().begin();
            }
            break;

        default :
            std::string msg = "Waypoint mode not defined yet.";
            throw std::runtime_error(msg);
        }

        auto msg = std::make_shared<sc::Message<std::tuple<size_t, size_t, bool>>>();
        size_t idx = std::distance(wp_list_.waypoints().begin(), wp_it_);
        msg->data = std::make_tuple(idx, wp_list_.waypoints().size(), done);
        wp_pub_status_->publish(msg);
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
