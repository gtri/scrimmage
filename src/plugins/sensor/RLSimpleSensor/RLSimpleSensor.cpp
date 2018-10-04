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

#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::RLSimpleSensor, RLSimpleSensor_plugin)

namespace scrimmage {
namespace sensor {

void RLSimpleSensor::get_observation(double *data, uint32_t beg_idx, uint32_t /*end_idx*/) {
    data[beg_idx] = parent_->state()->pos()(0);
    data[beg_idx + 1] = time_->t();
}

void RLSimpleSensor::set_observation_space() {
    const double inf = std::numeric_limits<double>::infinity();
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
}

} // namespace sensor
} // namespace scrimmage
