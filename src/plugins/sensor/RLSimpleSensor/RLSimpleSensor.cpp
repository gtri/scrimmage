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

#include <scrimmage/plugins/sensor/RLSimpleSensor/RLSimpleSensor.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/ExternalControl.pb.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Sensor, RLSimpleSensor, RLSimpleSensor_plugin)

scrimmage::MessagePtr<scrimmage_proto::SpaceSample>
RLSimpleSensor::sensor_msg_flat(double t) {
    auto msg = std::make_shared<sc::Message<sp::SpaceSample>>();
    msg->data.add_value(parent_->state()->pos()(0));
    return msg;
}

scrimmage_proto::SpaceParams RLSimpleSensor::observation_space_params() {
    sp::SpaceParams space_params;

    const double inf = std::numeric_limits<double>::infinity();
    sp::SingleSpaceParams *single_space_params = space_params.add_params();
    single_space_params->set_num_dims(1);
    single_space_params->add_minimum(-inf);
    single_space_params->add_maximum(inf);
    single_space_params->set_discrete(false);

    return space_params;
}
