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

#include <scrimmage/common/Random.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/StateWithCovariance.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/sensor/NoisyContacts/NoisyContacts.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::NoisyContacts, NoisyContacts_plugin)

namespace scrimmage {
namespace sensor {

NoisyContacts::NoisyContacts() {}

void NoisyContacts::init(std::map<std::string, std::string> &params) {
    auto add_noise = [&](std::string prefix, auto &noise_vec) {
        for (int i = 0; i < 3; i++) {
            std::string tag_name = prefix + "_" + std::to_string(i);
            std::vector<double> vec;
            double mean, stdev;
            std::tie(mean, stdev) =
                get_vec(tag_name, params, " ", vec, 2) ? std::make_pair(vec[0], vec[1]) : std::make_pair(0.0, 1.0);
            noise_vec.push_back(parent_->random()->make_rng_normal(mean, stdev));
        }
    };

    add_noise("pos_noise", pos_noise_);
    add_noise("vel_noise", vel_noise_);
    add_noise("orient_noise", orient_noise_);

    std::string topic_name = sc::get<std::string>("topic_name", params, "ContactsWithCovariances");
    pub_ = advertise("LocalNetwork", topic_name);
}

bool NoisyContacts::step() {
    auto gener = parent_->random()->gener();
    auto msg = std::make_shared<Message<ContactMap>>();

    // Create noisy versions of all contacts that aren't own vehicle
    for (auto &kv : (*parent_->contacts())) {
        // ignore own vehicle
        if (kv.second.id().id() == parent_->id().id()) {
            continue;
        }

        // Construct a StateWithCovariance.
        // Covar Matrix is 3x3 (x, y, z) with 5.0 along diagonal
        std::shared_ptr<StateWithCovariance> state =
            std::make_shared<StateWithCovariance>(*(kv.second.state()), 3, 3, 5.0);

        // Add noise to position and velocity
        for (int i = 0; i < 3; i++) {
            state->pos()(i) += (*pos_noise_[i])(*gener);
            state->vel()(i) += (*vel_noise_[i])(*gener);
        }

        // add noise in roll, pitch, yaw order
        state->quat() = state->quat() * Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener)) *
                        Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener)) *
                        Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener));

        // Construct a new Contact in place
        msg->data.emplace(kv.second.id().id(), Contact(kv.second.id(), std::static_pointer_cast<State>(state)));
    }

    // Publish the noisy contact map
    pub_->publish(msg);

    return true;
}
}  // namespace sensor
}  // namespace scrimmage
