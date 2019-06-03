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
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/sensor/NoisyState/NoisyState.h>
#include <scrimmage/math/StateWithCovariance.h>

#include <vector>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::NoisyState, NoisyState_plugin)

namespace scrimmage {
namespace sensor {

void NoisyState::init(std::map<std::string, std::string> &params) {
    auto add_noise = [&](std::string prefix, auto &noise_vec) {
        for (int i = 0; i < 3; i++) {
            std::string tag_name = prefix + "_" + std::to_string(i);
            std::vector<double> vec;
            double mean, stdev;
            std::tie(mean, stdev) = get_vec(tag_name, params, " ", vec, 2) ?
                std::make_pair(vec[0], vec[1]) : std::make_pair(0.0, 1.0);
            noise_vec.push_back(parent_->random()->make_rng_normal(mean, stdev));
        }
    };

    add_noise("pos_noise", pos_noise_);
    add_noise("vel_noise", vel_noise_);
    add_noise("orient_noise", orient_noise_);

    pub_ = advertise("LocalNetwork", "StateWithCovariance");

    // Move the enity's state pointer to use the state instance provided by
    // this plugin.
    parent_->state() = std::make_shared<State>();
    *(parent_->state()) = *(parent_->state_truth());

    return;
}

bool NoisyState::step() {
    auto gener = parent_->random()->gener();

    // Make a copy of the current ground truth state
    StateWithCovariance ns(*(parent_->state_truth()));

    auto msg = std::make_shared<Message<StateWithCovariance>>();

    for (int i = 0; i < 3; i++) {
        msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener);
        msg->data.vel()(i) = ns.vel()(i) + (*vel_noise_[i])(*gener);
    }

    // TODO: Test this math.
    // add noise in roll, pitch, yaw order
    msg->data.quat() = ns.quat()
        * Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener))
        * Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener))
        * Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener));

    pub_->publish(msg);

    // Update the entity's state.
    *(parent_->state()) = static_cast<sc::State>(msg->data);

    return true;
}
} // namespace sensor
} // namespace scrimmage
