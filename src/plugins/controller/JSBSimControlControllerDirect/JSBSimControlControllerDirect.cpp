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

#include <scrimmage/plugins/controller/JSBSimControlControllerDirect/JSBSimControlControllerDirect.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/ArduPilot/PwmState.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/State.h>

#include <iostream>

#include <boost/algorithm/string.hpp>


REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::JSBSimControlControllerDirect, JSBSimControlControllerDirect_plugin)

namespace scrimmage {
namespace controller {

void JSBSimControlControllerDirect::init(std::map<std::string,
                                         std::string> &params) {
	aileron_idx_ = vars_.declare(VariableIO::Type::aileron, VariableIO::Direction::Out);
	elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::Out);
	throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);
	rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::Out);
}

bool JSBSimControlControllerDirect::step(double t, double dt) {

	// TODO: make the servo mapping configurable
    auto d_state = scrimmage::State::cast<scrimmage::motion::PwmState>(desired_state_);
	if ( d_state ) {
		Eigen::VectorXd scaledInput(scrimmage::scale(d_state->pwm_input(), d_state->pwm_min(), d_state->pwm_max(), -1.0, 1.0));
		vars_.output(aileron_idx_, scaledInput[0]);
		vars_.output(elevator_idx_, -scaledInput[1]);
		vars_.output(rudder_idx_, scaledInput[3]);
		vars_.output(throttle_idx_, scrimmage::scale(scaledInput[2], -1.0, 1.0, 0.0, 1.0));
    } else {
        std::cout << "Unable to cast desired_state to PwmState" << std::endl;
    }

    return true;
}
} // namespace controller
} // namespace scrimmage
