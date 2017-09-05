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
#include <iostream>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <scrimmage/plugins/autonomy/Boids/Boids.h>

namespace sc = scrimmage;

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::Autonomy, Boids, Boids_plugin)

void Boids::init(std::map<std::string, std::string> &params) {

    show_shapes_ = sc::get("show_shapes", params, false);
    max_speed_ = sc::get<double>("max_speed", params, 21);
    
    w_align_ = sc::get("align_weight", params, 0.01);
    w_avoid_team_ = sc::get("avoid_team_weight", params, 0.95);    
    w_centroid_ = sc::get("centroid_weight", params, 0.05);

    w_avoid_nonteam_ = sc::get("avoid_nonteam_weight", params, 1.0);
    fov_el_ = sc::Angles::deg2rad(sc::get("fov_el", params, 90));
    fov_az_ = sc::Angles::deg2rad(sc::get("fov_az", params, 90));
    comms_range_ = sc::get("comms_range", params, 1000);

    sphere_of_influence_ = sc::get<double>("sphere_of_influence", params, 10);
    minimum_team_range_ = sc::get<double>("minimum_team_range", params, 5);
    minimum_nonteam_range_ = sc::get<double>("minimum_nonteam_range", params, 10);
    
    w_goal_ = sc::get<double>("goal_weight", params, 1.0);

    if (sc::get("use_initial_heading", params, false)) {
        Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*10000;
        Eigen::Vector3d unit_vector = rel_pos.normalized();
        unit_vector = state_->quat().rotate(unit_vector);
        goal_ = state_->pos() + unit_vector * rel_pos.norm();
    } else {
        std::vector<double> goal_vec;
        if (sc::get_vec<double>("goal", params, " ", goal_vec, 3)) {
            goal_ = sc::vec2eigen(goal_vec);
        }
    }

    desired_state_->vel() = Eigen::Vector3d::UnitX()*21;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);
}

bool Boids::step_autonomy(double t, double dt)
{
    shapes_.clear();
    
    // Find neighbors that are within field-of-view and within comms range
    std::vector<sc::ID> rtree_neighbors;
    rtree_->neighbors_in_range(state_->pos_const(), rtree_neighbors, comms_range_);

    // Remove neighbors that are not within field of view
    std::vector<sc::ID>::iterator it = rtree_neighbors.begin();
    while (it != rtree_neighbors.end()) {
        // Ignore own position / id
        if (it->id() == parent_->id().id()) {
            it = rtree_neighbors.erase(it);
        } else if (state_->InFieldOfView(*(*contacts_)[it->id()].state(), fov_az_, fov_el_)) {
            // The neighbor is "in front"
            ++it;
        } else {
            // The neighbor is "behind." Remove it.
            it = rtree_neighbors.erase(it);
        }
    }

    // move-to-goal behavior
    Eigen::Vector3d v_goal = (goal_ - state_->pos()).normalized();
    
    // Steer to avoid local neighbors
    // Align with neighbors
    // Cohesion: move towards average position of neighbors
    // (i.e., Find centroid of neighbors)
    double heading = 0;
    Eigen::Vector3d align(0,0,0);
    Eigen::Vector3d centroid(0,0,0);

    // Compute repulsion vector from each robot (team/nonteam)
    std::vector<Eigen::Vector3d> O_team_vecs;
    std::vector<Eigen::Vector3d> O_nonteam_vecs;

    for (sc::ID id : rtree_neighbors) {        
        bool is_team = (id.team_id() == parent_->id().team_id());
                
        sc::StatePtr other_state = (*contacts_)[id.id()].state();

        // Calculate vector pointing from own position to other
        Eigen::Vector3d diff = other_state->pos() - state_->pos();
        double dist = diff.norm();

        // Calculate magnitude of repulsion vector
        double min_range = is_team ? minimum_team_range_ : minimum_nonteam_range_;
        double O_mag = 0;
        if (dist > sphere_of_influence_) {
            O_mag = 0;
        } else if (min_range < dist && dist <= sphere_of_influence_) {
            O_mag = (sphere_of_influence_ - dist) /
                (sphere_of_influence_ - min_range);
        } else if (dist <= min_range) {
            O_mag = 1e10;
        }

        // Calculate repulsion vector
        Eigen::Vector3d O_dir = - O_mag * diff.normalized();
        if (is_team) {
            O_team_vecs.push_back(O_dir);
        } else {            
            O_nonteam_vecs.push_back(O_dir);
        }               
        
        // Calculate centroid of team members and heading alignment
        if (is_team) {
            centroid = centroid + other_state->pos();
            align += other_state->vel().normalized();
            heading += other_state->quat().yaw();
        }                        
    }

    Eigen::Vector3d align_vec(0,0,0);
    if (rtree_neighbors.size() > 0) {
        centroid = centroid / static_cast<double>(rtree_neighbors.size());
        align = align / static_cast<double>(rtree_neighbors.size());
        heading /= static_cast<double>(rtree_neighbors.size());        
        align_vec << cos(heading), sin(heading), 0;
    }

    // Make sure alignment vector is well-behaved
    align = align_vec; // TODO
    Eigen::Vector3d v_align_normed = align.normalized();
    double v_align_norm = align.norm();
    if (v_align_normed.hasNaN()) {
        v_align_normed = Eigen::Vector3d::Zero();
        v_align_norm = 0;
    }
    
    // Normalize each team repulsion vector and sum
    Eigen::Vector3d O_team_vec(0, 0, 0);
    for (Eigen::Vector3d v : O_team_vecs) {
        if (v.hasNaN()) {
            continue; // ignore misbehaved vectors
        }
        O_team_vec += v;
    }

    // Normalize each nonteam repulsion vector and sum
    Eigen::Vector3d O_nonteam_vec(0, 0, 0);
    for (Eigen::Vector3d v : O_nonteam_vecs) {
        if (v.hasNaN()) {
            continue; // ignore misbehaved vectors
        }
        O_nonteam_vec += v;
    }

    // Apply gains to independent behaviors
    Eigen::Vector3d v_goal_w_gain = v_goal * w_goal_;
    Eigen::Vector3d O_team_vec_w_gain = O_team_vec * w_avoid_team_;
    Eigen::Vector3d O_nonteam_vec_w_gain = O_nonteam_vec * w_avoid_nonteam_;
    Eigen::Vector3d v_centroid_w_gain = (centroid - state_->pos()).normalized() * w_centroid_;    
    Eigen::Vector3d v_align_w_gain = v_align_normed * w_align_;

    double sum_norms = v_goal_w_gain.norm() + O_team_vec_w_gain.norm() +
        O_nonteam_vec_w_gain.norm() + v_centroid_w_gain.norm() +
        v_align_norm;

    Eigen::Vector3d v_sum = (v_goal_w_gain + O_team_vec_w_gain +
                             O_nonteam_vec_w_gain + v_centroid_w_gain +
                             v_align_w_gain) / sum_norms;

    // Scale velocity to max speed:
    Eigen::Vector3d vel_result = v_sum * max_speed_;
    
    if (rtree_neighbors.size() > 0) {
        // Forward speed is normed value of vector:
        desired_state_->vel()(0) = vel_result.norm();

        // Desired heading
        double heading = sc::Angles::angle_2pi(atan2(vel_result(1), vel_result(0)));
        desired_state_->quat().set(0, 0, heading);

        // Set Desired Altitude by projecting velocity
        desired_state_->pos()(2) = state_->pos()(2) + vel_result(2);

        if (show_shapes_) {
            sc::ShapePtr shape(new scrimmage_proto::Shape);
            shape->set_type(scrimmage_proto::Shape::Sphere);
            shape->set_opacity(0.1);
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
    }

    return true;
}
