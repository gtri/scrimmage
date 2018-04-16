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

#include <scrimmage/plugins/autonomy/MoveToGoalMS/MoveToGoalMS.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::motor_schemas::MoveToGoalMS,
                MoveToGoalMS_plugin)

namespace scrimmage {
namespace autonomy {
namespace motor_schemas {

MoveToGoalMS::MoveToGoalMS() {
}

void MoveToGoalMS::init(std::map<std::string, std::string> &params) {
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

    auto goal_callback = [&] (scrimmage::MessagePtr<Eigen::Vector3d> msg) {
        goal_ = msg->data;
    };
    subscribe<Eigen::Vector3d>("LocalNetwork", "WaypointGoal", goal_callback);
}

bool MoveToGoalMS::step_autonomy(double t, double dt) {
    // move-to-goal schema
    desired_vector_ = (goal_ - state_->pos());

    // Normalize the vector if it greater than 1.0 in length. This allows us to
    // stop at goals
    if (desired_vector_.norm() > 1.0) {
        desired_vector_ = desired_vector_.normalized();
    } else {
    }
    return true;
}
} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage
