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

#include <scrimmage/plugins/controller/MultirotorControllerPID/MultirotorControllerPID.h>
#include <scrimmage/plugins/controller/MultirotorControllerOmega/MultirotorControllerOmega.h>
#include <scrimmage/plugins/motion/Multirotor/MultirotorState.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::MultirotorControllerPID,
                MultirotorControllerPID_plugin)

namespace scrimmage {
namespace controller {

MultirotorControllerPID::MultirotorControllerPID() {
}

void MultirotorControllerPID::init(std::map<std::string, std::string> &params) {
    multirotor_ = std::dynamic_pointer_cast<sc::motion::Multirotor>(parent_->motion());

    if (multirotor_ == nullptr) {
        cout << "WARNING: MultirotorControllerOmega can't control the motion "
             << "model for this entity." << endl;
        u_.resize(10);
        return;
    }

    u_.resize(multirotor_->rotors().size());
    u_ = Eigen::VectorXd::Zero(multirotor_->rotors().size());

    if (!sc::set_pid_gains(yaw_pid_, params["yaw_pid"], true)) {
        cout << "Failed to set MultirotorControllerPID yaw gains" << endl;
    }

    for (int i = 0; i < 3; i++) {
        vel_pids_.push_back(sc::PID());
        if (!sc::set_pid_gains(vel_pids_[i], params["vel_pid"])) {
            cout << "Failed to set DoubleIntegratorControllerVewYaw vel gain"
                 << endl;
        }
    }

    for (int i = 0; i < 3; i++) {
        angle_pids_.push_back(sc::PID());
        if (!sc::set_pid_gains(angle_pids_[i], params["angle_pid"])) {
            cout << "Failed to set DoubleIntegratorControllerVewYaw angle PID gains."
                 << endl;
        }
    }
}

bool MultirotorControllerPID::step(double t, double dt) {

    Eigen::Vector3d des_vel_body = state_->quat().rotate_reverse(desired_state_->vel());
    Eigen::Vector3d vel_body = state_->quat().rotate_reverse(state_->vel());
    Eigen::Vector3d vel_ctrl(0, 0, 0);
    for (int i = 0; i < 3; i++) {
        vel_pids_[i].set_setpoint(des_vel_body(i));
        vel_ctrl(i) = vel_pids_[i].step(dt, vel_body(i));
    }

    // cout << "Desired Velocity (Global): " << desired_state_->vel() << endl;
    // cout << "Desired Velocity (Body): " << des_vel_body << endl;

    for (unsigned int i = 0; i < multirotor_->rotors().size(); i++) {
        u_(i) = sqrt(multirotor_->mass() * multirotor_->gravity_magnitude() /
                     (multirotor_->rotors().size() * multirotor_->c_T()));
    }

    // double desired_roll = 0;
    // angle_pids_[0].set_setpoint(desired_roll);
    // double roll_ctrl = angle_pids_[0].step(dt, state_->quat().roll());
    //
    // double desired_pitch = 0.1;
    // angle_pids_[1].set_setpoint(desired_pitch);
    // double pitch_ctrl = angle_pids_[1].step(dt, state_->quat().pitch());
    //
    // double desired_yaw = 0;
    // angle_pids_[2].set_setpoint(desired_yaw);
    // double yaw_ctrl = angle_pids_[2].step(dt, state_->quat().yaw());
    //
    // u_(1) += pitch_ctrl * 1;
    // u_(2) += pitch_ctrl * 1;
    //
    // u_(0) -= pitch_ctrl * 1;
    // u_(3) -= pitch_ctrl * 1;


    // yaw_pid_.set_setpoint(desired_state_->quat().yaw());
    // double yaw_ctrl = yaw_pid_.step(dt, state_->quat().yaw());

    return true;
}
} // namespace controller
} // namespace scrimmage
