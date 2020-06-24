/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2019 by the Georgia Tech Research Institute (GTRI)
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
 * @author Joel Dunham <joel.dunham@gtri.gatech.edu>
 * @date 08 January 2019
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_LOSSENSOR_LOSSENSOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_LOSSENSOR_LOSSENSOR_H_

#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>

#include <eigen3/Eigen/Dense>

#include <map>
#include <string>
#include <vector>
#include <random>
#include <memory>

namespace scrimmage {
namespace sensor {
class LOSSensor : public RayTrace {
 public:
    LOSSensor();
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;

 private:
    // Sensor characteristics
    uint32_t sensor_id_;
    double last_update_time_;
    double range_sd_min_;
    double range_sd_per_unit_;
    double range_sd_oor_;
    double oor_return_;

    // Error characteristics
    double probability_of_error_;
    double error_sd_;
    double probability_oor_error_;

    // For simple tests
    bool use_flat_earth_;

    // For the collision capability
    bool new_data_;
    double range_;
    bool oor_;
    bool subscribed_;

    // For generating errors
    std::default_random_engine generator_;

    // The output
    PublisherPtr los_pub_;
};
}  // namespace sensor
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_LOSSENSOR_LOSSENSOR_H_
