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

#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>

#include <scrimmage/plugins/controller/JSBSimModelControllerDirect/JSBSimModelControllerDirect.h>

#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::JSBSimModelControllerDirect, JSBSimModelControllerDirect_plugin)

namespace scrimmage {
namespace controller {

namespace sc = scrimmage;
using ang = scrimmage::Angles;

void JSBSimModelControllerDirect::init(std::map<std::string, std::string> &params) {
    aircraft_ = std::dynamic_pointer_cast<sc::motion::JSBSimModel>(parent_->motion());
}

bool JSBSimModelControllerDirect::step(double t, double dt) {
    u_(0) = desired_state_->vel()(0);        // velocity
    u_(1) = desired_state_->quat().roll();   // bank
    if(aircraft_->use_pitch()){
        u_(2) = desired_state_->pos()(0);    // pitch
    }else{
        u_(2) = desired_state_->pos()(2);    // altitude
    }
    return true;
}
} // namespace controller
} // namespace scrimmage
