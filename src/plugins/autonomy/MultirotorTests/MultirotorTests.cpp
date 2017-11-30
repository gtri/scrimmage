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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::MultirotorTests,
                MultirotorTests_plugin)

namespace scrimmage {
namespace autonomy {

MultirotorTests::MultirotorTests() {
}

void MultirotorTests::init(std::map<std::string, std::string> &params) {
    multirotor_ = std::dynamic_pointer_cast<sc::motion::Multirotor>(parent_->motion());
    if (multirotor_ == nullptr) {
        cout << "WARNING: MultirotorTests can't control the motion "
             << "model for this entity." << endl;
    }

    desired_rotor_state_ = std::make_shared<sc::motion::MultirotorState>();
    desired_rotor_state_->set_type("MultirotorState");
    desired_rotor_state_->set_input_type(sc::motion::MultirotorState::InputType::PWM);
    desired_rotor_state_->prop_input().resize(multirotor_->rotors().size());

    double w = 1455; // 734.85
    for (int i = 0; i < desired_rotor_state_->prop_input().size(); i++) {
        desired_rotor_state_->prop_input()(i) = w;
    }

    desired_state_ = desired_rotor_state_;
}

bool MultirotorTests::step_autonomy(double t, double dt) {
    return true;
}
} // namespace autonomy
} // namespace scrimmage
