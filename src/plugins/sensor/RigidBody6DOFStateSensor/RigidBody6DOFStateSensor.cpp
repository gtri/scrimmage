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

#include <scrimmage/plugins/sensor/RigidBody6DOFStateSensor/RigidBody6DOFStateSensor.h>
#include <scrimmage/plugins/motion/RigidBody6DOF/RigidBody6DOFState.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/plugins/motion/Multirotor/Multirotor.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor,
                scrimmage::sensor::RigidBody6DOFStateSensor,
                RigidBody6DOFStateSensor_plugin)

namespace scrimmage {
namespace sensor {

RigidBody6DOFStateSensor::RigidBody6DOFStateSensor() {
}

void RigidBody6DOFStateSensor::init(std::map<std::string, std::string> &params) {
    // Use the same generator as the parent so that the simulation is
    // completely deterministic with respect to the simulation seed.
    gener_ = parent_->random()->gener();

    // Create three independent gaussian noise generators. They will use the
    // same generator seed.
    for (int i = 0; i < 3; i++) {
        std::string tag_name = "pos_noise_" + std::to_string(i);
        std::vector<double> vec;
        bool status = sc::get_vec(tag_name, params, " ", vec, 2);
        if (status) {
            pos_noise_.push_back(parent_->random()->make_rng_normal(vec[0], vec[1]));
        } else {
            pos_noise_.push_back(parent_->random()->make_rng_normal(0, 1));
        }
    }

    motion_ = std::dynamic_pointer_cast<scrimmage::motion::Multirotor>(parent_->motion());
    if (motion_ == nullptr) {
        cout << "WARNING: Failed to get motion model. Currently only "
             << "scrimmage::motion::Multirotor is supported by "
             << "RigidBody6DOFStateSensor" << endl;
    }

    return;
}

scrimmage::MessageBasePtr RigidBody6DOFStateSensor::sensor_msg(double t) {
    auto msg = std::make_shared<sc::Message<sc::motion::RigidBody6DOFState>>();

    // Copy elements from scrimmage::State
    msg->data.pos() = parent_->state()->pos();
    msg->data.vel() = parent_->state()->vel();
    msg->data.ang_vel() = parent_->state()->ang_vel();
    msg->data.quat() = parent_->state()->quat();

    msg->data.linear_vel_body() <<
        motion_->full_state_vector()[sc::motion::Multirotor::U],
        motion_->full_state_vector()[sc::motion::Multirotor::V],
        motion_->full_state_vector()[sc::motion::Multirotor::W];

    msg->data.ang_vel_body() <<
        motion_->full_state_vector()[sc::motion::Multirotor::P],
        motion_->full_state_vector()[sc::motion::Multirotor::Q],
        motion_->full_state_vector()[sc::motion::Multirotor::R];

    msg->data.linear_accel_body() <<
        motion_->full_state_vector()[sc::motion::Multirotor::U_dot],
        motion_->full_state_vector()[sc::motion::Multirotor::V_dot],
        motion_->full_state_vector()[sc::motion::Multirotor::W_dot];

    msg->data.ang_accel_body() <<
        motion_->full_state_vector()[sc::motion::Multirotor::P_dot],
        motion_->full_state_vector()[sc::motion::Multirotor::Q_dot],
        motion_->full_state_vector()[sc::motion::Multirotor::R_dot];


    // Return the sensor message.
    return msg;
}
} // namespace sensor
} // namespace scrimmage
