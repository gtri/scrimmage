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
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>

#include <iostream>
#include <limits>
#include <string>

#include <boost/algorithm/string/predicate.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::WayPointFollower, WayPointFollower_plugin)

namespace scrimmage {
namespace autonomy {

WayPointFollower::WayPointFollower(): wp_idx_(0), max_alt_change_(5.0),
    wp_tolerance_(10.0), returning_stage_(false) {
    waypoint_type_ = WayPointType::gps;
    waypoint_mode_ = WayPointMode::racetrack;
}

void WayPointFollower::init(std::map<std::string, std::string> &params) {

    double initial_speed = sc::get<double>("initial_speed", params, 21.0);
    max_alt_change_ = sc::get<double>("max_alt_change", params, 5.0);
    wp_tolerance_ = sc::get<double>("waypoint_tolerance", params, 10.0);
    bool show_waypoints = sc::get<bool>("show_waypoints", params, true);

    // coordinate frame of waypoints
    std::string type = sc::get<std::string>("waypoint_type", params, "gps");
    if (boost::iequals(type, "gps")) {
	waypoint_type_ = WayPointType::gps;
    } else if (boost::iequals(type, "cartesian")) {
	waypoint_type_ = WayPointType::cartesian;
    } else {
	std::string msg =
            "Need a waypoint type in WayPointFollower, got \"" + type + "\"";
        throw std::runtime_error(msg);
    }

    // waypoint following mode
    std::string mode = sc::get<std::string>("mode", params, "racetrack");
    if (boost::iequals(mode, "follow_once")) {
	waypoint_mode_ = WayPointMode::follow_once;
    } else if (boost::iequals(mode, "back_and_forth")) {
	waypoint_mode_ = WayPointMode::back_and_forth;
    } else if (boost::iequals(mode, "loiter")) {
	waypoint_mode_ = WayPointMode::loiter;
    } else if (boost::iequals(mode, "racetrack")) {
	waypoint_mode_ = WayPointMode::racetrack;
    } else {
	std::string msg =
            "Need a waypoint type in WayPointFollower, got \"" + mode + "\"";
        throw std::runtime_error(msg);
    }

    // read waypoints from xml file
    std::vector<std::string> wps_str;
    bool exist = sc::get_vec("waypoint_list/point", params, wps_str);
    if (!exist) {
	std::string msg = "Need to specify a list of waypoints.";
        throw std::runtime_error(msg);
    }

    for (std::string s : wps_str) {
	std::vector<double> vec;
	if (sc::str2vec(s, ",", vec, 3)) {

	    // make sure waypoints are in cartesian coordinates
	    Eigen::Vector3d p;
	    switch (waypoint_type_) {
	    case WayPointType::gps :
		parent_->projection()->Forward(vec[0], vec[1], vec[2],
					       p(0), p(1), p(2));
		break;
	    case WayPointType::cartesian :
		p << vec[0],
		    vec[1],
		    vec[2];
		break;
	    default :
		std::string msg = "This waypoint type is not yet defined.";
		throw std::runtime_error(msg);
	    }

	    // waypoint list
	    wps_.push_back(WayPoint(p, wp_tolerance_));

	    if (show_waypoints) {
		draw_waypoints(p);
	    }
	} else {
	    std::string msg = "Each waypoint value should be separated by a comma.";
	    throw std::runtime_error(msg);
	}
    }

    // define desired state
    desired_state_->vel() = Eigen::Vector3d::UnitX()*initial_speed;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    // create loiter waypoints around the last specified waypoint
    if (waypoint_mode_ == WayPointMode::loiter) {
	Eigen::Vector3d center = wps_.back().point;
	wps_.clear();
	double loiter_radius = 100;
	int num_slices = 10;
	std::vector<Eigen::Vector3d> xyz_points;
	for (int i = 0; i < num_slices; i++) {
	    double theta = i / static_cast<double>(num_slices) * 2 * M_PI;
	    Eigen::AngleAxis<double> rot(theta, Eigen::Vector3d::UnitZ());
	    Eigen::Vector3d v_rot = rot.toRotationMatrix() * Eigen::Vector3d::UnitX();
	    Eigen::Vector3d p = loiter_radius * v_rot + center;
	    xyz_points.push_back(p);
	    wps_.push_back(WayPoint(p, 50.0));
	}

	// find closest point on loiter radius and head to that waypoint index first
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

	if (show_waypoints) {
	    shapes_.clear();
	    draw_waypoints(center);
	}
    }
}


void WayPointFollower::draw_waypoints(Eigen::Vector3d v) {
    sc::ShapePtr shape(new scrimmage_proto::Shape);
    shape->set_type(scrimmage_proto::Shape::Sphere);
    shape->set_opacity(0.5);
    shape->set_radius(2.5);
    shape->set_persistent(true);
    sc::set(shape->mutable_center(), v(0), v(1), v(2));
    sc::set(shape->mutable_color(), 0, 255, 0);
    shapes_.push_back(shape);
}


bool WayPointFollower::step_autonomy(double t, double dt) {

    WayPoint curr_wp = wps_[wp_idx_];
    Eigen::Vector3d p = curr_wp.point;

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

    // check if within waypoint tolerance
    if ((state_->pos().head<2>() - p.head<2>()).norm() <= curr_wp.tolerance) {

	wp_idx_ = (returning_stage_) ? wp_idx_-1 : wp_idx_+1;

        if (wp_idx_ >= wps_.size() || wp_idx_ == 0) {
	    switch (waypoint_mode_) {
	    case WayPointMode::follow_once :
		return true;
		break;

	    case WayPointMode::back_and_forth :
		if (wp_idx_ == 0) {
		    returning_stage_ = false;
		} else {
		    returning_stage_ = true;
		    wp_idx_ -= 2;
		}
		break;

	    case WayPointMode::loiter :
		wp_idx_ = 0;
		break;

	    case WayPointMode::racetrack :
		wp_idx_ = 0;
		break;

	    default :
		std::string msg = "WayPoint mode not defined yet.";
		throw std::runtime_error(msg);
	    }
	}
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
