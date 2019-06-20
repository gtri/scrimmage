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

#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/Boids/Boids.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <vector>

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::Boids, Boids_plugin)

namespace sc = scrimmage;

namespace scrimmage {
namespace autonomy {

void Boids::init(std::map<std::string, std::string> &params) {

    show_shapes_ = get("show_shapes", params, false);
    max_speed_ = get<double>("max_speed", params, 21);

    w_align_ = get("align_weight", params, 0.01);
    w_avoid_team_ = get("avoid_team_weight", params, 0.95);
    w_centroid_ = get("centroid_weight", params, 0.05);

    w_avoid_nonteam_ = get("avoid_nonteam_weight", params, 1.0);
    fov_el_ = Angles::deg2rad(get("fov_el", params, 90));
    fov_az_ = Angles::deg2rad(get("fov_az", params, 90));
    comms_range_ = get("comms_range", params, 1000);

    sphere_of_influence_ = get<double>("sphere_of_influence", params, 10);
    minimum_team_range_ = get<double>("minimum_team_range", params, 5);
    minimum_nonteam_range_ = get<double>("minimum_nonteam_range", params, 10);

    w_goal_ = get<double>("goal_weight", params, 1.0);

    if (get("use_initial_heading", params, false)) {
        Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1000000;
        Eigen::Vector3d unit_vector = rel_pos.normalized();
        unit_vector = state_->quat().rotate(unit_vector);
        goal_ = state_->pos() + unit_vector * rel_pos.norm();
    } else {
        std::vector<double> goal_vec;
        if (get_vec<double>("goal", params, " ", goal_vec, 3)) {
            goal_ = vec2eigen(goal_vec);
        }
    }

    io_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
    io_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
    io_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);

    io_vel_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::Out);
    io_turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
    io_pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);

    io_desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    io_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
    io_altitude_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
}

bool Boids::step_autonomy(double t, double dt) {
    // Find neighbors that are within field-of-view and within comms range
    std::vector<ID> rtree_neighbors;
    rtree_->neighbors_in_range(state_->pos(), rtree_neighbors, comms_range_);

    // Remove neighbors that are not within field of view
    for (auto it = rtree_neighbors.begin(); it != rtree_neighbors.end();
         /* no inc */) {

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
    Eigen::Vector3d align(0, 0, 0);
    Eigen::Vector3d centroid(0, 0, 0);

    // Compute repulsion vector from each robot (team/nonteam)
    std::vector<Eigen::Vector3d> O_team_vecs;
    std::vector<Eigen::Vector3d> O_nonteam_vecs;

    for (ID id : rtree_neighbors) {
        bool is_team = (id.team_id() == parent_->id().team_id());

        StatePtr other_state = (*contacts_)[id.id()].state();

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

    Eigen::Vector3d align_vec(0, 0, 0);
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

        velocity_controller(vel_result);

        if (show_shapes_) {
            ShapePtr sphere(new scrimmage_proto::Shape);
            sphere->set_opacity(0.1);
            sc::set(sphere->mutable_color(), 0, 255, 0);
            sphere->mutable_sphere()->set_radius(sphere_of_influence_);
            sc::set(sphere->mutable_sphere()->mutable_center(), state_->pos());
            draw_shape(sphere);

            // Draw resultant vector:
            ShapePtr line(new scrimmage_proto::Shape);
            line->set_opacity(0.75);
            sc::set(line->mutable_color(), 255, 255, 0);
            sc::set(line->mutable_line()->mutable_start(), state_->pos());
            sc::set(line->mutable_line()->mutable_end(), vel_result + state_->pos());
            draw_shape(line);
        }
    } else {
        velocity_controller(v_goal);
    }
    return true;
}


void Boids::velocity_controller(Eigen::Vector3d &v) {
    // Convert to spherical coordinates:
    double desired_heading = atan2(v(1), v(0));
    double desired_pitch = atan2(v(2), v.head<2>().norm());

    vars_.output(io_vel_idx_, max_speed_);
    vars_.output(io_turn_rate_idx_, Angles::angle_pi(desired_heading - state_->quat().yaw()));
    vars_.output(io_pitch_rate_idx_, Angles::angle_pi(desired_pitch + state_->quat().pitch()));

    vars_.output(io_heading_idx_, desired_heading);
    vars_.output(io_altitude_idx_, v(2));
    vars_.output(io_desired_speed_idx_, max_speed_);

    double norm = v.norm();
    double ratio = (max_speed_ / 2) / std::max(norm, 1.0);
    if (norm != 0 && ratio < 1) {
        v *= ratio;
    }

    vars_.output(io_vel_x_idx_, v(0));
    vars_.output(io_vel_y_idx_, v(1));
    vars_.output(io_vel_z_idx_, v(2));
}
} // namespace autonomy
} // namespace scrimmage
