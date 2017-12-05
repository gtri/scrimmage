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

#include <scrimmage/plugins/autonomy/MultirotorTests/MultirotorTests.h>
#include <scrimmage/plugins/motion/RigidBody6DOF/RigidBody6DOFState.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/sensor/Sensor.h>

#include <iostream>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::MultirotorTests,
                MultirotorTests_plugin)

namespace scrimmage {
namespace autonomy {

MultirotorTests::MultirotorTests() {
    angles_to_gps_.set_input_clock_direction(sc::Angles::Rotate::CCW);
    angles_to_gps_.set_input_zero_axis(sc::Angles::HeadingZero::Pos_X);
    angles_to_gps_.set_output_clock_direction(sc::Angles::Rotate::CW);
    angles_to_gps_.set_output_zero_axis(sc::Angles::HeadingZero::Pos_Y);
}

void MultirotorTests::init(std::map<std::string, std::string> &params) {
    multirotor_ = std::dynamic_pointer_cast<sc::motion::Multirotor>(parent_->motion());
    if (multirotor_ == nullptr) {
        cout << "WARNING: MultirotorTests can't control the motion "
             << "model for this entity." << endl;
    }

    desired_rotor_state_ = std::make_shared<sc::motion::MultirotorState>();
    desired_rotor_state_->set_input_type(sc::motion::MultirotorState::InputType::PWM);
    desired_rotor_state_->prop_input().resize(multirotor_->rotors().size());

    double w = 1455; // 734.85
    for (int i = 0; i < desired_rotor_state_->prop_input().size(); i++) {
        desired_rotor_state_->prop_input()(i) = w;
    }
    desired_rotor_state_->prop_input()(0) += 10;

    desired_state_ = desired_rotor_state_;
}

bool MultirotorTests::step_autonomy(double t, double dt) {
    sc::motion::RigidBody6DOFState state;
    for (auto kv : parent_->sensors()) {
        if (kv.first == "RigidBody6DOFStateSensor0") {
            auto msg = kv.second->sense<sc::motion::RigidBody6DOFState>(t);
            if (msg) {
                state = (*msg)->data;
            }
        }
    }

    // simulation time in microseconds
    uint64_t timestamp_us = t * 1e6;

    // x/y/z to lat/lon/alt conversion
    double latitude, longitude, altitude;
    parent_->projection()->Reverse(state.pos()(0), state.pos()(1),
                                   state.pos()(2), latitude, longitude,
                                   altitude);

    // Heading conversion
    angles_to_gps_.set_angle(sc::Angles::rad2deg(state.quat().yaw()));
    double heading = angles_to_gps_.angle();

    double speedN = state.vel()(1);
    double speedE = state.vel()(0);
    double speedD = -state.vel()(2);

    // Body frame linear acceleration
    double xAccel = state.linear_accel_body()(0);
    double yAccel = -state.linear_accel_body()(1);
    double zAccel = -state.linear_accel_body()(2);

    // Body frame rotational accelerations
    double rollRate = state.ang_accel_body()(0);
    double pitchRate = state.ang_accel_body()(1);
    double yawRate = state.ang_accel_body()(2);

    // Global frame, roll, pitch, yaw
    double roll = state.quat().roll();
    double pitch = state.quat().pitch();
    double yaw = state.quat().yaw(); // Is this the same as heading?

    // Airspeed is magnitude of velocity vector for now
    double airspeed = state.vel().norm();

    cout << "---------------" << endl;
    cout << "timestamp_us: " << timestamp_us << endl;
    cout << "Latitude: " << latitude << endl;
    cout << "Longitude: " << longitude << endl;
    cout << "Altitude: " << altitude << endl;
    cout << "Airspeed: " << airspeed << endl;
    cout << "Roll: " << roll << endl;
    cout << "Pitch: " << pitch << endl;
    cout << "Yaw: " << yaw << endl;
    cout << "xAccel: " << xAccel << endl;
    cout << "yAccel: " << yAccel << endl;
    cout << "zAccel: " << zAccel << endl;
    cout << "speedN: " << speedN << endl;
    cout << "speedE: " << speedE << endl;
    cout << "speedD: " << speedD << endl;
    cout << "heading: " << heading << endl;
    cout << "rollRate: " << rollRate << endl;
    cout << "pitchRate: " << pitchRate << endl;
    cout << "yawRate: " << yawRate << endl;

    return true;
}
} // namespace autonomy
} // namespace scrimmage
