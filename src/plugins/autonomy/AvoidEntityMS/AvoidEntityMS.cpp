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

#include <scrimmage/plugins/autonomy/AvoidEntityMS/AvoidEntityMS.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::motor_schemas::AvoidEntityMS,
                AvoidEntityMS_plugin)

namespace scrimmage {
namespace autonomy {
namespace motor_schemas {

AvoidEntityMS::AvoidEntityMS() : sphere_of_influence_(10.0),
                                 minimum_range_(5.0),
                                 avoid_non_team_(true) {
}

void AvoidEntityMS::init(std::map<std::string, std::string> &params) {
    sphere_of_influence_ = sc::get<double>("sphere_of_influence", params, 10);
    minimum_range_ = sc::get<double>("minimum_range", params, 5);
    avoid_non_team_ = sc::get<bool>("avoid_non_team", params, true);
    show_shapes_ = sc::get<bool>("show_shapes", params, false);

    configure_contacts(params);
}

void AvoidEntityMS::avoidance_vectors(ContactMap &contacts,
                                      std::vector<Eigen::Vector3d> &O_vecs) {
    for (auto it = contacts.begin(); it != contacts.end(); it++) {
        // Ignore own position / id
        if (it->second.id().id() == parent_->id().id()) {
            continue;
        }

        if (!avoid_non_team_ &&
            it->second.id().team_id() != parent_->id().team_id()) {
            continue;
        }

        Eigen::Vector3d diff = it->second.state()->pos() - state_->pos();

        double O_mag = 0;
        double dist = diff.norm();

        if (dist > sphere_of_influence_) {
            O_mag = 0;
        } else if (minimum_range_ < dist && dist <= sphere_of_influence_) {
            O_mag = (sphere_of_influence_ - dist) /
                (sphere_of_influence_ - minimum_range_);
        } else if (dist <= minimum_range_) {
            O_mag = 1e10;
        }

        Eigen::Vector3d O_dir = -O_mag * diff.normalized();
        O_vecs.push_back(O_dir);
    }
}

bool AvoidEntityMS::step_autonomy(double t, double dt) {
    // Compute repulsion vector from each robot contact
    std::vector<Eigen::Vector3d> O_vecs;

    if (use_truth_contacts_) {
        avoidance_vectors(*contacts_, O_vecs);
    }
    if (use_noisy_contacts_) {
        avoidance_vectors(noisy_contacts_, O_vecs);
    }

    // Normalize each repulsion vector and sum
    desired_vector_ <<  0, 0, 0;
    for (auto it = O_vecs.begin(); it != O_vecs.end(); it++) {
        if (it->hasNaN()) {
            continue; // ignore misbehaved vectors
        }
        desired_vector_ += *it;
    }

    desired_vector_ *= max_vector_length_;

    if (show_shapes_) {
        // Draw the sphere of influence
        if (circle_shape_ == nullptr) {
            circle_shape_ = sc::shape::make_circle(
                state_->pos(), state_->quat(), sphere_of_influence_,
                Eigen::Vector3d(0, 255, 0), 0.2);
        }
        sc::set(circle_shape_->mutable_circle()->mutable_center(), state_->pos());
        draw_shape(circle_shape_);

        if (line_shape_ == nullptr) {
            line_shape_ = sc::shape::make_line(
                state_->pos(), desired_vector_ + state_->pos(),
                Eigen::Vector3d(0, 0, 0), 0.75);
        }
        sc::set(line_shape_->mutable_line()->mutable_start(), state_->pos());
        sc::set(line_shape_->mutable_line()->mutable_end(), desired_vector_ + state_->pos());
        draw_shape(line_shape_);
    }

    return true;
}
} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage
