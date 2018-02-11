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
#include <scrimmage/proto/ExternalControl.pb.h>

#include <scrimmage/plugins/sensor/TutorialOpenAISensor/TutorialOpenAISensor.h>

#include <boost/range/adaptor/map.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace ba = boost::adaptors;

REGISTER_PLUGIN(scrimmage::Sensor, TutorialOpenAISensor, TutorialOpenAISensor_plugin)

scrimmage::MessagePtr<scrimmage_proto::SpaceSample>
TutorialOpenAISensor::sensor_msg_flat(double t) {
    auto msg = std::make_shared<sc::Message<sp::SpaceSample>>();

    // we need these sorted but contacts are an unordered map
    auto keys = *parent_->contacts() | ba::map_keys;
    std::set<int> contact_ids(keys.begin(), keys.end());

    for (int contact_id : contact_ids) {
        sc::State &s = *parent_->contacts()->at(contact_id).state();
        const double yaw = s.quat().yaw();
        msg->data.add_value(s.pos()(0));
        msg->data.add_value(s.pos()(1));
        msg->data.add_value(cos(yaw));
        msg->data.add_value(sin(yaw));
    }

    return msg;
}

scrimmage_proto::SpaceParams TutorialOpenAISensor::observation_space_params() {
    sp::SpaceParams space_params;

    const double inf = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < parent_->contacts()->size(); i++) {
        sp::SingleSpaceParams *single_space_params = space_params.add_params();
        single_space_params->set_num_dims(4);

        const std::vector<double> lims {inf, inf, 1, 1}; // x, y, cos(yaw), sin(yaw)
        for (double lim : lims) {
          single_space_params->add_minimum(-lim);
          single_space_params->add_maximum(lim);
        }
        single_space_params->set_discrete(false);
    }

    return space_params;
}
