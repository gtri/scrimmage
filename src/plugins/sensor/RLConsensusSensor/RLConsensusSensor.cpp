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

#include <scrimmage/plugins/sensor/RLConsensusSensor/RLConsensusSensor.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>

#include <iostream>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/copy.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace ba = boost::adaptors;
namespace br = boost::range;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::RLConsensusSensor, RLConsensusSensor_plugin)

namespace scrimmage {
namespace sensor {

RLConsensusSensor::RLConsensusSensor() : ScrimmageOpenAISensor() {}

void RLConsensusSensor::set_observation_space() {
    const int num_veh = parent_->contacts()->size();
    const double inf = std::numeric_limits<double>::infinity();
    observation_space.continuous_extrema.assign(num_veh, std::make_pair(-inf, inf));
}

void RLConsensusSensor::get_observation(double *data, uint32_t beg_idx, uint32_t end_idx) {
    auto c = parent_->contacts();
    if (c->size() != end_idx - beg_idx) {
        std::cout << "RLConsensusSensor::get_observation (end_idx - beg_idx) "
            << "does not match number of vehicles" << std::endl;
        return;
    }

    auto ids_view = *c | ba::map_keys;
    std::set<int> ids(ids_view.begin(), ids_view.end());

    auto get_x = [&](int id) {return c->at(id).state()->pos()(0);};
    br::copy(ids | ba::transformed(get_x), data);
}
} // namespace sensor
} // namespace scrimmage
