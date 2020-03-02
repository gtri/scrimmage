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
 * @author Natalie Rakoski <natalie.rakoski@gtri.gatech.edu>
 * @date 27 January 2020
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSCOMPASS_ROSCOMPASS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSCOMPASS_ROSCOMPASS_H_

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <scrimmage/sensor/Sensor.h>

#include <random>
#include <vector>
#include <map>
#include <string>
#include <memory>

namespace scrimmage {
namespace sensor {

class ROSCompass : public scrimmage::Sensor {
 public:
    ROSCompass();
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;

 protected:
    std::string ros_namespace_;
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher compass_pub_;

 private:
};
} // namespace sensor
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSCOMPASS_ROSCOMPASS_H_
