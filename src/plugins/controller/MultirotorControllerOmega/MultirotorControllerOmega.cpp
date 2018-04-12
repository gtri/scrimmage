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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/controller/MultirotorControllerOmega/MultirotorControllerOmega.h>
#include <scrimmage/plugins/motion/Multirotor/MultirotorState.h>
#include <scrimmage/plugins/autonomy/ArduPilot/PwmState.h>

#include <iostream>
#include <limits>
#include <typeinfo>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage::motion;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::MultirotorControllerOmega,
                MultirotorControllerOmega_plugin)

namespace scrimmage {
namespace controller {

MultirotorControllerOmega::MultirotorControllerOmega() : pwm_max_(2000),
                                                         pwm_min_(1000) {
}

void MultirotorControllerOmega::init(std::map<std::string, std::string> &params) {

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
}

bool MultirotorControllerOmega::step(double t, double dt) {
    auto d_state = sc::State::cast<sc::motion::MultirotorState>(desired_state_);
    auto d_state2 = sc::State::cast<sc::motion::PwmState>(desired_state_);
    if (d_state) {
        if (d_state->input_type() == sm::MultirotorState::InputType::OMEGA) {
            u_ = d_state->prop_input().head(multirotor_->rotors().size());
        } else if (d_state->input_type() == sm::MultirotorState::InputType::PWM) {
            u_ = sc::scale(d_state->prop_input().head(multirotor_->rotors().size()), pwm_min_, pwm_max_,
                           multirotor_->omega_min(),
                           multirotor_->omega_max());
        } else {
            cout << "WARNING: Invalid MultirotorState input type" << endl;
        }
    } else if ( d_state2 ) {
        u_ = sc::scale(d_state2->pwm_input().head(multirotor_->rotors().size()), pwm_min_, pwm_max_,
                       multirotor_->omega_min(),
                       multirotor_->omega_max());
    } else {
        cout << "Unable to cast desired_state to MultirotorState" << endl;
        u_ = Eigen::VectorXd::Zero(multirotor_->rotors().size());
    }
    return true;
}
} // namespace controller
} // namespace scrimmage
