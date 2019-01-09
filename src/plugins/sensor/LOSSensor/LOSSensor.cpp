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

#include <scrimmage/common/Random.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/sensor/LOSSensor/LOSSensor.h>
#include <scrimmage/msgs/LOSSensor.pb.h>

#include <cstdlib>
#include <vector>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::LOSSensor, LOSSensor_plugin)

namespace scrimmage {
namespace sensor {


LOSSensor::LOSSensor():
    sensor_id_(0),
    min_sensor_range_(0.0),
    max_sensor_range_(0.0),
    range_sd_min_(0.0),
    range_sd_per_unit_(0.0),
    range_sd_oor_(0.0),
    oor_return_(0.1),
    probability_of_error_(0.0),
    error_sd_(0.0),
    probability_oor_error_(0.0) {
}

void LOSSensor::init(std::map<std::string, std::string> &params) {
    // Initialize the random number generator
    std::srand(std::time(0));

    // Get the parameters
    // Offset
    std::vector<double> vec;
    if (!get_vec("offset", params, " ", vec, 3)) {
        // No offset
        vec.clear();
        vec.push_back(0.0);
        vec.push_back(0.0);
        vec.push_back(0.0);
    }
    offset_ << vec[0], vec[1], vec[2];

    // Orientation
    vec.clear();
    if (!get_vec("orientation", params, " ", vec, 3)) {
        // Default is out the nose
        vec.clear();
        vec.push_back(1.0);
        vec.push_back(0.0);
        vec.push_back(0.0);
    }
    orientation_ << vec[0], vec[1], vec[2];

    sensor_id_ = sc::get<int>("sensor_id", params, 0);
    min_sensor_range_ = sc::get<double>("min_range", params, 0.0);
    max_sensor_range_ = sc::get<double>("max_range", params, 0.0);
    range_sd_min_ = sc::get<double>("range_sd_min", params, 0.0001);
    range_sd_per_unit_ = sc::get<double>("range_sd_per_unit", params, 0.0);
    range_sd_oor_ = sc::get<double>("range_sd_oor", params, 0.0001);
    oor_return_ = sc::get<double>("oor_return", params, 0.1);
    probability_of_error_ = sc::get<double>("probability_of_error", params, 0.0);
    error_sd_ = sc::get<double>("error_sd", params, 0.0001);
    probability_oor_error_ = sc::get<double>("probability_oor_error", params, 0.0);

    // Create the response
    pub_ = advertise("LocalNetwork", "LOSRange");
    return;
}

bool LOSSensor::step() {

    // Currently, there is no terrain or line-of-sight services.
    // Doing a simple geometry calculation to determine when/if the sensor
    //  impacts the terrain.  Range limits and noise added subsequently.
    // Note that this also does not handle other entities.  Will need a service
    //  to incorporate that.

    // Coordinate systems used:
    // Local level: East/North/Up
    // Body: Nose/Left/Up

    bool oor = false;
    double range = 0.0;

    // Get required state data
    Eigen::Vector3d pos = parent_->state()->pos();
    Quaternion quat = parent_->state()->quat();

    // Get the sensor location in the local level frame.
    Eigen::Vector3d offset_ll = quat.rotate_reverse(offset_);
    Eigen::Vector3d sensor_loc_ll = pos + offset_ll;

    // Get the sensor LOS unit vector in the local level frame.
    Eigen::Vector3d sensor_los_ll = quat.rotate_reverse(orientation_);

    // Verify that this will intersect the ground
    if ((sensor_loc_ll(2) < 0.0) || (sensor_los_ll(2) >= 0.0)) { // Out of range
        oor = true;
    } else { // Potentially in range
        // Project the unit vector intersection to the ground.
        range = sensor_loc_ll(2) / (-1.0 * sensor_los_ll(2));
        // Check the random error to see whether it could swap in-range/out-of-range.
        bool swap_in_out_range =
            (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX)) < probability_oor_error_;

        if ((((range < min_sensor_range_) || (range > max_sensor_range_)) && !swap_in_out_range) ||
            (((range > min_sensor_range_) && (range < max_sensor_range_)) && swap_in_out_range)) {
            // Out of range
            oor = true;
        } else { // In-range
            // Limit to the range boundaries (in case an error caused an in-range
            //  measurement when it was actually out of range)
            range = MIN(MAX(range, min_sensor_range_), max_sensor_range_);
        }
    }

    // Now that in-range versus out-of-range is known as well as the actual range,
    //  add error.
    auto msg = std::make_shared<sc::Message<sm::LOSRangeMessage>>();
    msg->data.set_sensor_id(sensor_id_);
    if (oor == false) {
        // Add error
        if ((static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX)) < probability_of_error_) {
            // Add additional error
            std::normal_distribution<double> error_dist(0.0, error_sd_);
            double error = error_dist(generator_);
            range = MIN(MAX(range + error, min_sensor_range_), max_sensor_range_);
        }

        // Finally, add the actual noise
        double total_sd = range_sd_min_ + (range_sd_per_unit_ * range);
        if (total_sd > 0.0) {
            std::normal_distribution<double> noise_dist(0.0, total_sd);
            range = MIN(MAX(range + noise_dist(generator_), min_sensor_range_), max_sensor_range_);
        }
        // Set the values to the message
        msg->data.set_range(range);
        msg->data.set_range_var(total_sd * total_sd);
    } else { // Out of range
        // Set the values to the message
        msg->data.set_range(oor_return_);
        msg->data.set_range_var(range_sd_oor_ * range_sd_oor_);
    }

    // Publish the message and return all good
    pub_->publish(msg);
    return true;
}
} // namespace sensor
} // namespace scrimmage
