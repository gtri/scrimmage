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

#include "scrimmage/plugins/sensor/NoisyState/NoisyState.h"

#include <vector>

#include "scrimmage/common/KeyedRandom.h"
#include "scrimmage/common/Random.h"
#include "scrimmage/common/Time.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/math/Quaternion.h"
#include "scrimmage/math/State.h"
#include "scrimmage/math/StateWithCovariance.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/RegisterPlugin.h"
#include "scrimmage/pubsub/Message.h"
#include "scrimmage/pubsub/Publisher.h"

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::NoisyState, NoisyState_plugin)

namespace scrimmage {
namespace sensor {

void NoisyState::init(std::map<std::string, std::string>& params) {
    auto add_noise = [&](std::string prefix, auto& noise_vec) {
        for (int i = 0; i < 3; i++) {
            std::string tag_name = prefix + "_" + std::to_string(i);
            std::vector<double> vec;
            double mean, stdev;
            std::tie(mean, stdev) = get_vec(tag_name, params, " ", vec, 2)
                                        ? std::make_pair(vec[0], vec[1])
                                        : std::make_pair(0.0, 1.0);
            noise_vec.push_back(parent_->random()->make_rng_normal(mean, stdev));
        }
    };

    add_noise("pos_noise", pos_noise_);
    add_noise("vel_noise", vel_noise_);
    add_noise("orient_noise", orient_noise_);

    pub_ = advertise("LocalNetwork", "StateWithCovariance");

    // The first belief is a reading too, sampled like every later one.
    parent_->set_state_belief(static_cast<sc::State>(sample(1)->data));

    return;
}

bool NoisyState::step() {
    auto msg = sample(0);

    pub_->publish(msg);

    // Update the entity's state.
    parent_->set_state_belief(static_cast<sc::State>(msg->data));

    return true;
}

std::shared_ptr<Message<StateWithCovariance>> NoisyState::sample(uint64_t reading) {
    auto gener = parent_->random()->gener();
    // scrimmage-rs comparison: with SCRIMMAGE_KEYED_RANDOM=1, each error is
    // the keyed normal number for [reading, quantity, axis], as in Ripple.
    const bool keyed = keyed_random::enabled();
    const uint64_t at_step = keyed ? keyed_random::plugin_label(
                                         parent_->random()->get_seed(), parent_->id().id(),
                                         random_identity(), time_->step())
                                   : 0;
    auto error = [&](uint64_t quantity, int axis, std::normal_distribution<double>& noise) {
        if (!keyed) {
            return noise(*gener);
        }
        const uint64_t label =
            keyed_random::key(at_step, {reading, quantity, static_cast<uint64_t>(axis)});
        return keyed_random::normal(label, noise.mean(), noise.stddev());
    };

    // Make a copy of the current ground truth state
    StateWithCovariance ns(*(parent_->state_truth()));

    auto msg = std::make_shared<Message<StateWithCovariance>>();
    // There is no rate noise: angular velocity is reported exactly.
    msg->data.ang_vel() = ns.ang_vel();

    for (int i = 0; i < 3; i++) {
        msg->data.pos()(i) = ns.pos()(i) + error(0, i, *pos_noise_[i]);
        msg->data.vel()(i) = ns.vel()(i) + error(1, i, *vel_noise_[i]);
    }

    // TODO: Test this math.
    // add noise in roll, pitch, yaw order
    if (!keyed) {
        msg->data.quat() = ns.quat() * Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener))
                           * Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener))
                           * Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener));
    } else {
        msg->data.quat() = ns.quat() * Quaternion(Eigen::Vector3d::UnitX(), error(2, 0, *orient_noise_[0]))
                           * Quaternion(Eigen::Vector3d::UnitY(), error(2, 1, *orient_noise_[1]))
                           * Quaternion(Eigen::Vector3d::UnitZ(), error(2, 2, *orient_noise_[2]));
    }

    return msg;
}

}  // namespace sensor
}  // namespace scrimmage
