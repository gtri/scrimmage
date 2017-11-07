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
#include <scrimmage/plugins/sensor/NoisyState/NoisyState.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>

#include <vector>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::NoisyState, NoisyState_plugin)

namespace scrimmage {
namespace sensor {

void NoisyState::init(std::map<std::string, std::string> &params) {
    gener_ = parent_->random()->gener();

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

    for (int i = 0; i < 3; i++) {
        std::string tag_name = "vel_noise_" + std::to_string(i);
        std::vector<double> vec;
        bool status = sc::get_vec(tag_name, params, " ", vec, 2);
        if (status) {
            vel_noise_.push_back(parent_->random()->make_rng_normal(vec[0], vec[1]));
        } else {
            vel_noise_.push_back(parent_->random()->make_rng_normal(0, 1));
        }
    }

    for (int i = 0; i < 3; i++) {
        std::string tag_name = "orient_noise_" + std::to_string(i);
        std::vector<double> vec;
        bool status = sc::get_vec(tag_name, params, " ", vec, 2);
        if (status) {
            orient_noise_.push_back(parent_->random()->make_rng_normal(vec[0], vec[1]));
        } else {
            orient_noise_.push_back(parent_->random()->make_rng_normal(0, 1));
        }
    }

    return;
}

boost::optional<scrimmage::MessageBasePtr> NoisyState::sensor_msg(double t) {
    // Make a copy of the current state
    sc::State ns = *(parent_->state());

    auto msg = std::make_shared<sc::Message<sc::State>>();

    for (int i = 0; i < 3; i++) {
        msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener_);
        msg->data.vel()(i) = ns.vel()(i) + (*vel_noise_[i])(*gener_);
    }

    // TODO: Test this math.
    // add noise in roll, pitch, yaw order
    msg->data.quat() = ns.quat()
        * sc::Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener_))
        * sc::Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener_))
        * sc::Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener_));

    return boost::optional<scrimmage::MessageBasePtr>(msg);
}
} // namespace sensor
} // namespace scrimmage
