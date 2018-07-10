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
#include <scrimmage/common/RTree.h>
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

#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/irange.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace ba = boost::adaptors;
namespace br = boost::range;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::NoisyContacts, NoisyContacts_plugin)

namespace scrimmage {
namespace sensor {

void NoisyContacts::init(std::map<std::string, std::string> &params) {
    gener_ = parent_->random()->gener();

    max_detect_range_ = sc::get<double>("max_detect_range", params, 1000);
    az_thresh_ = sc::Angles::deg2rad(sc::get<double>("azimuth_fov", params, 360));
    el_thresh_ = sc::Angles::deg2rad(sc::get<double>("elevation_fov", params, 360));

    auto make_rng = [&](auto tag_name) {
        std::vector<double> vec;
        bool status = sc::get_vec(tag_name, params, " ", vec, 2);
        return status ?
            parent_->random()->make_rng_normal(vec[0], vec[1]) :
            parent_->random()->make_rng_normal(0, 1);
    };

    auto make_vec = [&](auto prefix, auto &vec) {
        auto create_tag = [&](int i) {return prefix + std::to_string(i);};
        auto tag_names = boost::irange(0, 3) | ba::transformed(create_tag);
        br::transform(tag_names, std::back_inserter(vec), make_rng);
    };

    make_vec("pos_noise_", pos_noise_);
    make_vec("vel_noise_", vel_noise_);
    make_vec("pos_noise_", orient_noise_);

    return;
}

scrimmage::MessageBasePtr NoisyContacts::sensor_msg(double t) {
    auto msg = std::make_shared<sc::Message<std::list<sc::Contact>>>();

    auto state = parent_->state();
    auto contacts = parent_->contacts();

    auto in_fov = [&](const auto &id) {
        return state->InFieldOfView(*contacts->at(id.id()).state(), az_thresh_, el_thresh_);
    };

    auto create_noisy_contact = [&](const auto &id) {
        sc::Contact &c = contacts->at(id.id());
        sc::Contact noisy_c;
        noisy_c.set_id(id);

        for (int i = 0; i < 3; i++) {
            noisy_c.state()->pos()(i) = c.state()->pos()(i) + (*pos_noise_[i])(*gener_);
            noisy_c.state()->vel()(i) = c.state()->vel()(i) + (*vel_noise_[i])(*gener_);
        }

        // TODO: Test this math.
        // add noise in roll, pitch, yaw order
        noisy_c.state()->quat() = c.state()->quat()
            * sc::Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener_))
            * sc::Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener_))
            * sc::Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener_));

        return noisy_c;
    };

    std::vector<ID> neigh_in_range;
    parent_->rtree()->neighbors_in_range(
        state->pos(), neigh_in_range, max_detect_range_, parent_->id().id());

    br::transform(neigh_in_range | ba::filtered(in_fov),
                  std::inserter(msg->data, msg->data.end()),
                  create_noisy_contact);

    return msg;
}

} // namespace sensor
} // namespace scrimmage
