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

#include <scrimmage/common/ID.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/sensor/NoisyContacts/NoisyContacts.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/pubsub/Message.h>

#include <vector>

#include <boost/optional.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::NoisyContacts, NoisyContacts_plugin)

namespace scrimmage {
namespace sensor {

void NoisyContacts::init(std::map<std::string, std::string> &params) {
    gener_ = parent_->random()->gener();

    max_detect_range_ = sc::get<double>("max_detect_range", params, 1000);
    az_thresh_ = sc::Angles::deg2rad(sc::get<double>("azimuth_fov", params, 360));
    el_thresh_ = sc::Angles::deg2rad(sc::get<double>("elevation_fov", params, 360));

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

boost::optional<scrimmage::MessageBasePtr> NoisyContacts::sensor_msg(double t) {
    auto msg = std::make_shared<sc::Message<std::list<sc::Contact>>>();

    for (auto &kv : *(parent_->contacts())) {
        // Filter out (skip) own contact
        if (kv.second.id().id() == parent_->id().id()) continue;

        // Filter out contacts out of range, should use RTree, but rtree still
        // requires querying of ID from contact lists. (TODO)
        if ((kv.second.state()->pos() - parent_->state()->pos()).norm() >
            max_detect_range_) {
            continue;
        }

        // Filter out contacts out of FOV
        if (!parent_->state()->InFieldOfView(*(kv.second.state()), az_thresh_,
                                            el_thresh_)) {
            continue;
        }

        // Create noisy copy of contact
        sc::Contact c;
        c.set_id(kv.second.id());

        for (int i = 0; i < 3; i++) {
            c.state()->pos()(i) = kv.second.state()->pos()(i) + (*pos_noise_[i])(*gener_);
            c.state()->vel()(i) = kv.second.state()->vel()(i) + (*vel_noise_[i])(*gener_);
        }

        // TODO: Test this math.
        // add noise in roll, pitch, yaw order
        c.state()->quat() = kv.second.state()->quat()
            * sc::Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener_))
            * sc::Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener_))
            * sc::Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener_));

        msg->data.push_back(c);
    }

    return boost::optional<sc::MessageBasePtr>(msg);
}

#if ENABLE_GRPC == 1
boost::optional<scrimmage::MessagePtr<scrimmage_proto::SpaceSample>>
NoisyContacts::sensor_msg_flat(double t) {
    auto msg = std::make_shared<sc::Message<sp::SpaceSample>>();
    auto add_vec = [&](auto &vec) {
        msg->data.add_value(vec(0));
        msg->data.add_value(vec(1));
        msg->data.add_value(vec(2));
    };

    for (auto &kv : *parent_->contacts()) {
        sc::State &s = *kv.second.state();
        add_vec(s.pos());
        add_vec(s.vel());
        msg->data.add_value(s.quat().roll());
        msg->data.add_value(s.quat().pitch());
        msg->data.add_value(s.quat().yaw());
    }

    return msg;
}

boost::optional<scrimmage_proto::SpaceParams> NoisyContacts::observation_space_params() {
    sp::SpaceParams space_params;

    const double inf = std::numeric_limits<double>::infinity();
    for (size_t contact_num = 0; contact_num < parent_->contacts()->size(); contact_num++) {
        sp::SingleSpaceParams *single_space_params = space_params.add_params();
        single_space_params->set_num_dims(9);

        // position/velocity
        for (int i = 0; i < 6; i++) {
            single_space_params->add_minimum(-inf);
            single_space_params->add_maximum(inf);
        }

        // euler angles (could possibly do quaternion)
        // roll
        single_space_params->add_minimum(-M_PI / 2);
        single_space_params->add_maximum(M_PI / 2);

        // pitch
        single_space_params->add_minimum(-M_PI / 2);
        single_space_params->add_maximum(M_PI / 2);

        // yaw
        single_space_params->add_minimum(-M_PI);
        single_space_params->add_maximum(M_PI);

        single_space_params->set_discrete(false);
    }

    return space_params;
}
#endif
} // namespace sensor
} // namespace scrimmage
