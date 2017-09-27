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

#include <scrimmage/plugins/autonomy/WayPointFollower/WayPointFollower.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::cerr;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, WayPointFollower, WayPointFollower_plugin)

WayPointFollower::WayPointFollower(): staging_achieved_(false), wp_idx_(0),
    max_alt_change_(5.0), wp_tolerance_(10.0), loiter_mode_(false) {
}

void WayPointFollower::init(std::map<std::string, std::string> &params) {

    double initial_speed = sc::get<double>("initial_speed", params, 21.0);
    max_alt_change_ = sc::get<double>("max_alt_change", params, 5.0);
    wp_tolerance_ = sc::get<double>("waypoint_tolerance", params, 10.0);
    waypoint_type_ = sc::get<std::string>("waypoint_type", params, "gps");
    std::string mode = sc::get<std::string>("mode", params, "racetrack");

    std::vector<std::string> wps_str;
    bool exist = sc::get_vec("waypoint_list/point", params, wps_str);
    if (!exist) {
	cerr << "Need to specify a list of waypoints." << endl;
	exit(EXIT_FAILURE);
    }

    // organize waypoints in vectors
    for (std::string s : wps_str) {
	std::vector<double> vec;
	if (sc::str2vec(s, ",", vec, 3)) {
	    wps_.push_back(WayPoint(Eigen::Vector3d(vec[0], vec[1], vec[2]),
				    wp_tolerance_));
	} else {
	    cerr << "Each waypoint should be separated by a comma." << endl;
	    exit(EXIT_FAILURE);
	}
    }

    desired_state_->vel() = Eigen::Vector3d::UnitX()*initial_speed;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    // TODO: determine waypoint following mode with the enum class
    if (mode == "loiter") {
        loiter_mode_ = true;
    }

    // TODO: improve waypoint definition for loiter mode
    if (loiter_mode_) {
	// Create loiter waypoints around position
        Eigen::Vector3d center;
        parent_->projection()->Forward(34.3871, -103.3116, 1000,
				       center(0), center(1), center(2));
        double loiter_radius = 3000;
        int num_slices = 100;
        std::vector<Eigen::Vector3d> xyz_points;
        for (int i = 0; i < num_slices; i++) {
            double theta = i / static_cast<double>(num_slices) * 2 * M_PI;
            Eigen::AngleAxis<double> rot(theta, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d v_rot = rot.toRotationMatrix() * Eigen::Vector3d::UnitX();
            Eigen::Vector3d p = loiter_radius * v_rot + center;
            xyz_points.push_back(p);

            Eigen::Vector3d lla;
            parent_->projection()->Reverse(p(0), p(1), p(2), lla(0), lla(1),
                                           lla(2));
            wps_.push_back(WayPoint(lla, wp_tolerance_));
        }

        // Find closest point on loiter radius and head to that waypoint index
        // first:
        int i = 0;
        int champ_index = i;
        double min_dist = std::numeric_limits<double>::infinity();
        for (Eigen::Vector3d p : xyz_points) {
            double dist = (p - state_->pos()).norm();
            if (dist < min_dist) {
                min_dist = dist;
                champ_index = i;
            }
            i++;
        }
        wp_idx_ = champ_index;
    }
}

bool WayPointFollower::step_autonomy(double t, double dt) {

    WayPoint curr_wp = wps_[wp_idx_];

    // get current waypoint in local cartesian coordinates
    Eigen::Vector3d p;
    if (waypoint_type_.compare("gps") == 0) {
	parent_->projection()->Forward(curr_wp.point(0),
				       curr_wp.point(1),
				       curr_wp.point(2),
				       p(0), p(1), p(2));
    } else if (waypoint_type_.compare("cartesian") == 0) {
	p << curr_wp.point(0),
	     curr_wp.point(1),
	     curr_wp.point(2);
    }

    // set altitude
    double alt_diff = p(2) - state_->pos()(2);
    if (std::abs(alt_diff) > max_alt_change_) {
        if (alt_diff > 0) {
            desired_state_->pos()(2) = state_->pos()(2) + max_alt_change_;
        } else {
            desired_state_->pos()(2) = state_->pos()(2) - max_alt_change_;
        }
    } else {
        desired_state_->pos()(2) = p(2);
    }

    double heading = atan2(p(1)- state_->pos()(1),
                           p(0)- state_->pos()(0));

    // set the heading
    desired_state_->quat().set(0, 0, heading);

    // cout << "Dist: " << (state_->pos().head<2>() - p.head<2>()).norm() << endl;
    // Check if within waypoint tolerance:
    if ((state_->pos().head<2>() - p.head<2>()).norm() <= curr_wp.tolerance) {

        if (!staging_achieved_) {
            staging_achieved_ = true;
        } else {
            wp_idx_++;
        }

        if (wp_idx_ >= wps_.size()) {
            wp_idx_ = 0;
        }
    }

    return true;
}
