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

#include <scrimmage/plugins/autonomy/WaypointFollower/WaypointFollower.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointList.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <iostream>
using std::endl;
using std::cout;

#include <limits>

#include <boost/algorithm/string/predicate.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::WaypointFollower, WaypointFollower_plugin)

namespace scrimmage {
namespace autonomy {

void WaypointFollower::init(std::map<std::string, std::string> &params) {

    double initial_speed = sc::get<double>("initial_speed", params, 21.0);
    max_alt_change_ = sc::get<double>("max_alt_change", params, 5.0);
    exit_on_reaching_wpt_ = sc::str2bool(params.at("exit_on_reaching_wpt"));

    // define desired state
    desired_state_->vel() = Eigen::Vector3d::UnitX() * initial_speed;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    auto wp_list_cb = [&] (scrimmage::MessagePtr<WaypointList> msg) {
        // Received a new waypoint list, reset
        wp_list_ = msg->data;
        wp_idx_ = 0;
    };
    subscribe<WaypointList>("GlobalNetwork", "WaypointList", 10, wp_list_cb);

    // TODO: Probably should be move loiter behavior into own plugin
    // // create loiter waypoints around the last specified waypoint
    // if (waypoint_mode_ == WaypointMode::loiter) {
    //     Eigen::Vector3d center = wps_.back().point;
    //     wps_.clear();
    //     double loiter_radius = 100;
    //     int num_slices = 10;
    //     std::vector<Eigen::Vector3d> xyz_points;
    //     for (int i = 0; i < num_slices; i++) {
    //         double theta = i / static_cast<double>(num_slices) * 2 * M_PI;
    //         Eigen::AngleAxis<double> rot(theta, Eigen::Vector3d::UnitZ());
    //         Eigen::Vector3d v_rot = rot.toRotationMatrix() * Eigen::Vector3d::UnitX();
    //         Eigen::Vector3d p = loiter_radius * v_rot + center;
    //         xyz_points.push_back(p);
    //         wps_.push_back(Waypoint(p, 50.0));
    //     }
    //
    //     // find closest point on loiter radius and head to that waypoint index first
    //     int i = 0;
    //     int champ_index = i;
    //     double min_dist = std::numeric_limits<double>::infinity();
    //     for (Eigen::Vector3d p : xyz_points) {
    //         double dist = (p - state_->pos()).norm();
    //         if (dist < min_dist) {
    //         min_dist = dist;
    //         champ_index = i;
    //         }
    //         i++;
    //     }
    //     wp_idx_ = champ_index;
    //
    //     if (show_waypoints) {
    //         shapes_.clear();
    //         draw_waypoints(center, waypoint_color, waypoint_radius);
    //     }
    // }
}

bool WaypointFollower::step_autonomy(double t, double dt) {
    if (wp_list_.waypoints().size() == 0) {
        return true;
    }

    Waypoint curr_wp = wp_list_.waypoints()[wp_idx_];

    Eigen::Vector3d wp;
    parent_->projection()->Forward(curr_wp.latitude(), curr_wp.longitude(),
                                   curr_wp.altitude(), wp(0), wp(1), wp(2));

    // set altitude
    double alt_diff = wp(2) - state_->pos()(2);
    desired_state_->pos().head<2>() = wp.head<2>();
    if (std::abs(alt_diff) > max_alt_change_) {
        if (alt_diff > 0) {
            desired_state_->pos()(2) = state_->pos()(2) + max_alt_change_;
        } else {
            desired_state_->pos()(2) = state_->pos()(2) - max_alt_change_;
        }
    } else {
        desired_state_->pos()(2) = wp(2);
    }

    double heading = atan2(wp(1)- state_->pos()(1),
                           wp(0)- state_->pos()(0));

    // set the heading
    desired_state_->quat().set(0, 0, heading);

    // check if within waypoint tolerance
    if ((state_->pos().head<2>() - wp.head<2>()).norm() <= curr_wp.position_tolerance()) {
        if (exit_on_reaching_wpt_) {
            return false;
        }

        wp_idx_ = (returning_stage_) ? wp_idx_-1 : wp_idx_+1;

        if (wp_idx_ >= wp_list_.waypoints().size() || wp_idx_ == 0) {
            switch (wp_list_.mode()) {
                case WaypointList::WaypointMode::follow_once :
                    wp_idx_ = wp_list_.waypoints().size()-1;
                    return true;
                    break;

                case WaypointList::WaypointMode::back_and_forth :
                    if (wp_idx_ == 0) {
                        returning_stage_ = false;
                    } else {
                        returning_stage_ = true;
                        wp_idx_ -= 2;
                    }
                    break;

                case WaypointList::WaypointMode::loiter :
                    wp_idx_ = 0;
                    break;

                case WaypointList::WaypointMode::racetrack :
                    wp_idx_ = 0;
                    break;

                default :
                    std::string msg = "Waypoint mode not defined yet.";
                    throw std::runtime_error(msg);
            }
        }
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
