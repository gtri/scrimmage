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

#include <Eigen/Geometry>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>

#include <scrimmage/plugins/autonomy/MotorSchemas/MotorSchemas.h>

#include <cmath>
#include <cfloat>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::MotorSchemas, MotorSchemas_plugin)

namespace scrimmage {
namespace autonomy {

void MotorSchemas::init(std::map<std::string, std::string> &params) {
    show_shapes_ = sc::get("show_shapes", params, false);

    max_speed_ = sc::get<double>("max_speed", params, 21);

    move_to_goal_gain_ = sc::get<double>("move_to_goal_gain", params, 1.0);

    sqrt_axis_ratio_ = std::sqrt(sc::get<double>("axis_ratio", params, 1.0));

    if (sc::get("use_initial_heading", params, false)) {
        Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*2000;
        Eigen::Vector3d unit_vector = rel_pos.normalized();
        unit_vector = state_->quat().rotate(unit_vector);
        goal_ = state_->pos() + unit_vector * rel_pos.norm();
    } else {
        std::vector<double> goal_vec;
        if (sc::get_vec<double>("goal", params, " ", goal_vec, 3)) {
            goal_ = sc::vec2eigen(goal_vec);
        }
    }

    avoid_robot_gain_ = sc::get<double>("avoid_robot_gain", params, 1.0);
    sphere_of_influence_ = sc::get<double>("sphere_of_influence", params, 10);
    minimum_range_ = sc::get<double>("minimum_range", params, 5);

    desired_state_->vel() = Eigen::Vector3d::UnitX()*21;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);
}

bool MotorSchemas::step_autonomy(double t, double dt) {
    shapes_.clear();

    // move-to-goal schema
    Eigen::Vector3d v_goal = (goal_ - state_->pos()).normalized();

    // Compute repulsion vector from each robot contact
    std::vector<Eigen::Vector3d> O_vecs;

    double yaw = state_->quat().yaw();

    Eigen::AngleAxisd rot(yaw, Eigen::Vector3d(0.0, 0.0, 1.0));
    Eigen::DiagonalMatrix<double, 3> d(sqrt_axis_ratio_, 1.0 / sqrt_axis_ratio_, 1.0);
    auto transform = rot*d*rot.inverse();
    auto transform_inv = transform.inverse();

    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {

        // Ignore own position / id
        if (it->second.id().id() == parent_->id().id()) continue;

        Eigen::Vector3d diff = it->second.state()->pos() - state_->pos();
        Eigen::Vector3d diff_rot = transform_inv*diff;

        double O_mag = 0;
        double dist = diff_rot.norm();

        if (dist > sphere_of_influence_) {
            O_mag = 0;
        } else if (minimum_range_ < dist && dist <= sphere_of_influence_) {
            O_mag = (sphere_of_influence_ - dist) /
                (sphere_of_influence_ - minimum_range_);
        } else if (dist <= minimum_range_) {
            O_mag = 1e10;
        }

        Eigen::Vector3d O_dir = - O_mag * diff.normalized();
        O_vecs.push_back(O_dir);
    }

    // Normalize each repulsion vector and sum
    Eigen::Vector3d O_vec(0, 0, 0);
    for (auto it = O_vecs.begin(); it != O_vecs.end(); it++) {
        if (it->hasNaN()) {
            continue; // ignore misbehaved vectors
        }
        O_vec += *it;
    }

    // Apply gains to independent behaviors
    Eigen::Vector3d v_goal_w_gain = v_goal * move_to_goal_gain_;
    Eigen::Vector3d O_vec_w_gain = O_vec * avoid_robot_gain_;

    // Sum vectors across behaviors
    double sum_norms = v_goal_w_gain.norm() + O_vec_w_gain.norm();
    Eigen::Vector3d v_sum =  (v_goal_w_gain + O_vec_w_gain) / sum_norms;

    // Scale velocity to max speed:
    Eigen::Vector3d vel_result = v_sum * max_speed_;

    ///////////////////////////////////////////////////////////////////////////
    // Convert resultant vector into heading / speed / altitude command:
    ///////////////////////////////////////////////////////////////////////////

    // Forward speed is normed value of vector:
    desired_state_->vel()(0) = vel_result.norm();

    // Desired heading
    double heading = sc::Angles::angle_2pi(atan2(vel_result(1), vel_result(0)));
    desired_state_->quat().set(0, 0, heading);

    // Set Desired Altitude by projecting velocity
    desired_state_->pos()(2) = state_->pos()(2) + vel_result(2);

    ///////////////////////////////////////////////////////////////////////////
    // Draw important shapes
    ///////////////////////////////////////////////////////////////////////////
    // Draw sphere of influence:
    if (show_shapes_) {
        sc::ShapePtr shape(new scrimmage_proto::Shape);
        shape->set_type(scrimmage_proto::Shape::Circle);
        shape->set_opacity(0.2);
        shape->set_radius(sphere_of_influence_);
        sc::set(shape->mutable_center(), state_->pos());
        sc::set(shape->mutable_color(), 0, 255, 0);
        shapes_.push_back(shape);

        // Draw resultant vector:
        sc::ShapePtr arrow(new scrimmage_proto::Shape);
        arrow->set_type(scrimmage_proto::Shape::Line);
        sc::set(arrow->mutable_color(), 255, 255, 0);
        arrow->set_opacity(0.75);
        sc::add_point(arrow, state_->pos());
        sc::add_point(arrow, vel_result + state_->pos());
        shapes_.push_back(arrow);
    }
    return true;
}
} // namespace autonomy
} // namespace scrimmage
