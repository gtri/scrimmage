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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_ROSIMUSENSOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_ROSIMUSENSOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/math/Quaternion.h>

#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/common/CSV.h>

#include <random>
#include <vector>
#include <map>
#include <string>
#include <memory>

namespace scrimmage {
namespace sensor {

class ROSIMUSensor : public scrimmage::Sensor {
 public:
    ROSIMUSensor();
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;
    void close(double t) override;

 protected:
    std::string ros_namespace_;
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher imu_pub_;

    Eigen::Vector3d prev_vel_;
    Quaternion prev_quat_;
    double prev_time_ = 0.0;

    scrimmage::CSV csv;

 private:
};
} // namespace sensor
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_ROSIMUSENSOR_H_
