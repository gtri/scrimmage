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

#include <scrimmage/plugins/autonomy/TrailMS/TrailMS.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/math/Angles.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::TrailMS,
                TrailMS_plugin)

namespace scrimmage {
namespace autonomy {

void TrailMS::init(std::map<std::string, std::string> &params) {
    trail_id_ = sc::get<int>("trail_id", params, trail_id_);
    trail_range_ = sc::get<double>("trail_range", params, trail_range_);
    show_track_point_ = sc::get<bool>("show_track_point", params, show_track_point_);

    double trail_angle_az = sc::Angles::deg2rad(sc::get<double>("trail_angle_azimuth", params, M_PI));
    double trail_angle_elev = sc::Angles::deg2rad(sc::get<double>("trail_angle_elevation", params, 0));
    aa_angle_az_ = Eigen::AngleAxisd(trail_angle_az, Eigen::Vector3d::UnitZ());
    aa_angle_elev_ = Eigen::AngleAxisd(trail_angle_elev, Eigen::Vector3d::UnitY());
}

bool TrailMS::step_autonomy(double t, double dt) {
    if (trail_id_ < 0) return true;

    // Get state of vehicle that we are trailing
    sc::StatePtr trail_state;
    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
        if (trail_id_ == it->second.id().id()) {
            trail_state = it->second.state();
            break;
        }
    }

    // Compute the direction of the trail point relative to the other entity's
    // center. Rotate a unit vector pointing towards the x-axis around the
    // z-axis by the trail angle. Then rotate that vector by the other entity's
    // orientation.
    Eigen::Vector3d direction = trail_state->quat() * aa_angle_elev_ *
        aa_angle_az_ * Eigen::Vector3d::UnitX();

    // Compute the position of the desired trail point
    Eigen::Vector3d trail_point = trail_state->pos() + direction * trail_range_;

    // Compute the positio of the desisired trail point, taking the other
    // entity's velocity into consideration
    Eigen::Vector3d trail_point_w_vel = trail_point + trail_state->vel();

    // Compute the vector pointing from our position to the trail_point_w_vel
    desired_vector_ = trail_point_w_vel - state_->pos();

    // Normalize the vector to the maximum length
    if (desired_vector_.norm() > max_vector_length_) {
        desired_vector_ = desired_vector_.normalized() * max_vector_length_;
    }

    // Decrease speed as we get closer to the actual track point
    double trail_point_diff = (state_->pos() - trail_point).norm();
    if (trail_point_diff < desired_vector_.norm()) {
        double speed = desired_vector_.norm() - (state_->vel().norm() - trail_state->vel().norm());
        desired_vector_ = desired_vector_.normalized() * speed;
    }

    if (show_track_point_) {
        if (sphere_shape_ == nullptr) {
            sphere_shape_ = sc::shape::make_sphere(trail_point, 0.5,
                                                   Eigen::Vector3d(0, 0, 255));
        }
        sc::set(sphere_shape_->mutable_sphere()->mutable_center(), trail_point);
        draw_shape(sphere_shape_);
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
