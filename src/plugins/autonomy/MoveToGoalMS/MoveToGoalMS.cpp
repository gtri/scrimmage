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
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <limits>
#include <cmath>
#include <list>

#include <GeographicLib/LocalCartesian.hpp>

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
        Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
        Eigen::Vector3d unit_vector = rel_pos.normalized();
        unit_vector = state_->quat().rotate(unit_vector);
        wp_local_ = state_->pos() + unit_vector * rel_pos.norm();
    } else {
        std::vector<double> goal_vec;
        if (sc::get_vec<double>("goal", params, ", ", goal_vec, 3)) {
            wp_local_ = sc::vec2eigen(goal_vec);
        } else {
            cout << "Failed to parse MoveToGoalMS' initial goal" << endl;
        }
    }

    // Initialize PID controller class
    speed_pid_.set_parameters(sc::get("p_gain", params, 0.5),
                              sc::get("i_gain", params, 0.0),
                              sc::get("d_gain", params, 0.0));
    speed_pid_.set_integral_band(sc::get("integral_band", params, 0.0));
    speed_pid_.set_setpoint(0.0);

    // Convert XYZ goal to lat/lon/alt
    double lat, lon, alt;
    parent_->projection()->Reverse(wp_local_(0), wp_local_(1),
                                   wp_local_(2), lat, lon, alt);

    wp_ = Waypoint(lat, lon, alt);
    wp_.set_time(0);
    wp_.set_quat(scrimmage::Quaternion(0, 0, 0));
    wp_.set_position_tolerance(1);
    wp_.set_quat_tolerance(1);

    auto wp_cb = [&] (scrimmage::MessagePtr<Waypoint> msg) {
        wp_ = msg->data;
        parent_->projection()->Forward(wp_.latitude(),
                                       wp_.longitude(),
                                       wp_.altitude(), wp_local_(0),
                                       wp_local_(1), wp_local_(2));
    };
    subscribe<Waypoint>("LocalNetwork", "Waypoint", wp_cb);
}

bool MoveToGoalMS::step_autonomy(double t, double dt) {
    double measurement = -(wp_local_ - state_->pos()).norm();
    double speed_factor = speed_pid_.step(dt, measurement);
    desired_vector_ = (wp_local_ - state_->pos()).normalized() * speed_factor;
    return true;
}
} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage
