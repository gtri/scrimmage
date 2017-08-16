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

#include <scrimmage/sensor/Sensor.h>

#include <boost/optional.hpp>

namespace scrimmage {

void Sensor::init(std::map<std::string, std::string> &params) {return;}

std::string Sensor::name() {return std::string("Sensor");}
std::string Sensor::type() {return std::string("Sensor");}

boost::optional<scrimmage_proto::SpaceParams> Sensor::observation_space_params() {
    return boost::none;
}

boost::optional<scrimmage::MessageBasePtr> Sensor::sensor_msg(double t) {
    return boost::none;
}

boost::optional<scrimmage::MessagePtr<scrimmage_proto::SpaceSample>>
Sensor::sensor_msg_flat(double t) {
    return boost::none;
}

} // namespace scrimmage
