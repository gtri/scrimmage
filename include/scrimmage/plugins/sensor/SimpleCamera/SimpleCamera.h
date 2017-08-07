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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_SIMPLECAMERA_SIMPLECAMERA_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_SIMPLECAMERA_SIMPLECAMERA_H_

#include <scrimmage/sensor/Sensor.h>

#include <map>
#include <string>

class SimpleCamera : public scrimmage::Sensor {
 public:
    virtual void init(std::map<std::string, std::string> &params);
    virtual boost::optional<scrimmage::MessageBasePtr> sensor_msg(double t);

 protected:
    double range_ = 0;
    double fov_az_ = 0;
    double fov_el_ = 0;
    bool draw_cone_ = false;
};

#endif // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_SIMPLECAMERA_SIMPLECAMERA_H_
