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

#include <scrimmage/plugins/sensor/AltitudeAboveTerrain/AltitudeAboveTerrain.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <iostream>
#include <limits>

#include <boost/optional.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Sensor,
                scrimmage::sensor::AltitudeAboveTerrain,
                AltitudeAboveTerrain_plugin)

namespace scrimmage {
namespace sensor {

AltitudeAboveTerrain::AltitudeAboveTerrain() {
}

void AltitudeAboveTerrain::init(std::map<std::string, std::string> &params) {
    // Use the same generator as the parent so that the simulation is
    // completely deterministic with respect to the simulation seed.
    gener_ = parent_->random()->gener();

    std::vector<double> vec;
    if (str2container(sc::get<std::string>("altitude_noise", params, "0.0, 1.0"),
                    ", ", vec, 2)) {
        noise_ = parent_->random()->make_rng_normal(vec[0], vec[1]);
    } else {
        noise_ = parent_->random()->make_rng_normal(0.0, 1.0);
    }

    pub_true_ = advertise("GlobalNetwork", "/" + std::to_string(parent_->id().id()) + "/TrueAltitudeAboveTerrain");
    pub_noise_ = advertise("LocalNetwork", "AltitudeAboveTerrain");

    auto terrain_map_cb = [&] (auto &msg) {
        map_ = std::make_shared<scrimmage::interaction::TerrainMap>(msg->data);
    };
    subscribe<scrimmage_msgs::Terrain>("GlobalNetwork", "Terrain", terrain_map_cb);
}

bool AltitudeAboveTerrain::step() {
    // Wait until we have received a valid terrain map
    if (map_ == nullptr) return true;

    boost::optional<double> height = map_->height_at(parent_->state_truth()->pos()(0),
                                                     parent_->state_truth()->pos()(1));
    if (height) {
        double alt = parent_->state_truth()->pos()(2) - *height;

        // Create and send the true altitude
        auto msg_true = std::make_shared<sc::Message<double>>();
        msg_true->data = alt;
        pub_true_->publish(msg_true);

        // Generate a noisy version of the altitude
        auto msg_noise = std::make_shared<sc::Message<double>>();
        msg_noise->data = alt + (*noise_)(*gener_);
        pub_noise_->publish(msg_noise);
    } else {
        std::cout << "AltitudeAboveTerrain: vehicle out of terrain map bounds" << endl;
    }
    return true;
}
} // namespace sensor
} // namespace scrimmage
