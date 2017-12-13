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
#include <scrimmage/plugins/motion/Multirotor/MultirotorControllerOmega/MultirotorControllerOmega.h>
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
    pwm_max_ = sc::get<int>("pwm_max", params, 2000);
    pwm_min_ = sc::get<int>("pwm_min", params, 1000);

    multirotor_ = std::dynamic_pointer_cast<sc::motion::Multirotor>(parent_->motion());

    if (multirotor_ == nullptr) {
        cout << "WARNING: MultirotorControllerOmega can't control the motion "
             << "model for this entity." << endl;
        u_.resize(10);
        return;
    }

    u_.resize(multirotor_->rotors().size());
    u_ = Eigen::VectorXd::Zero(multirotor_->rotors().size());
}

bool MultirotorControllerPID::step(double t, double dt) {
    double desired_heading = desired_state_->quat().yaw();
    Eigen::Vector3d desired_vel = desired_state_->vel();
    cout << "Desired heading: " << desired_heading << endl;
    cout << "Desired vel vec: " << desired_vel << endl;
    return true;
}
} // namespace controller
} // namespace scrimmage
