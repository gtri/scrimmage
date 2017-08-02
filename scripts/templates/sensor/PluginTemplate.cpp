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

#include <iostream>
#include <limits>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>

#include <(>>>PROJECT_NAME<<<)/plugins/sensor/(>>>PLUGIN_NAME<<<)/(>>>PLUGIN_NAME<<<).h>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, (>>>PLUGIN_NAME<<<), (>>>PLUGIN_NAME<<<)_plugin)

(>>>PLUGIN_NAME<<<)::(>>>PLUGIN_NAME<<<)()
{
}

void (>>>PLUGIN_NAME<<<)::init(std::map<std::string,std::string> &params)
{
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
        
    return;
}

sc::MessageBasePtr (>>>PLUGIN_NAME<<<)::sensor_msg(double t, bool &valid)
{
    // Make a copy of the current state
    sc::State ns = *(parent_->state());

    // Create a message to hold the modified state
    auto msg = std::make_shared<sc::Message<sc::State>>();

    // Add noise to the three scalars in the 3D position vector.
    for (int i = 0; i < 3; i++) {
        msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener_);    
    }

    // Set "valid" to true if this is a valid sensor measurement. If the
    // developer wanted to limit sensor sampling, the time variable, t, could
    // be used and valid could be set to false if the sampling is occurring too
    // frequently.
    valid = true;

    // Return the sensor message.
    return msg;
}

