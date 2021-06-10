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
#include <scrimmage/common/Time.h>
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
    RayTrace(),
    sensor_id_(0),
    last_update_time_(0.0),
    range_sd_min_(0.0),
    range_sd_per_unit_(0.0),
    range_sd_oor_(0.0),
    oor_return_(0.1),
    probability_of_error_(0.0),
    error_sd_(0.0),
    probability_oor_error_(0.0),
    use_flat_earth_(true),
    new_data_(false),
    range_(0.0),
    oor_(false),
    subscribed_(false) {
}

void LOSSensor::init(std::map<std::string, std::string> &params) {
    // Call the super class
    RayTrace::init(params);

    // Initialize the random number generator
    std::srand(std::time(0));

    // Get the parameters
    use_flat_earth_ = sc::get<bool>("use_flat_earth", params, false);
    sensor_id_ = sc::get<int>("sensor_id", params, 0);
    range_sd_min_ = sc::get<double>("range_sd_min", params, 0.0001);
    range_sd_per_unit_ = sc::get<double>("range_sd_per_unit", params, 0.0);
    range_sd_oor_ = sc::get<double>("range_sd_oor", params, 0.0001);
    oor_return_ = sc::get<double>("oor_return", params, 0.1);
    probability_of_error_ = sc::get<double>("probability_of_error", params, 0.0);
    error_sd_ = sc::get<double>("error_sd", params, 0.0001);
    probability_oor_error_ = sc::get<double>("probability_oor_error", params, 0.0);

    // Create the response
    std::string topic_name = "LOSRange" + std::to_string(sensor_id_);
    los_pub_ = advertise("LocalNetwork", topic_name);
    return;
}

bool LOSSensor::step() {
    // Call the super class first
    bool retVal = RayTrace::step();
    if (retVal == false) {
        return retVal;
    }
    // Check whether an update is available if not using the collision service
    if (use_flat_earth_ == true) {
        if (time_->t() < (last_update_time_ + max_sample_rate())) {
            // No update, no message sent
            return retVal;
        }
    } else {  // Updated rate managed by collision system
        if (subscribed_ == false) {
            subscribed_ = true;
            // Register for the collision callbacks.  Assume set up for local network publish
            auto pc_cb = [&] (scrimmage::MessagePtr<sensor::RayTrace::PointCloud> msg) {
                range_ = msg->data.points[0].point.norm();
                oor_ = msg->data.points[0].oor;
                new_data_ = true;
            };
            std::string topic_name = std::to_string(parent_->id().id()) + "/" + name() + "/pointcloud";
            subscribe<sensor::RayTrace::PointCloud>("LocalNetwork", topic_name, pc_cb);
            printf("LOSSensor: Subscribing with topic name %s\n", topic_name.c_str());
        }
        if (new_data_ == false) {
            return retVal;
        }
    }

    // Can update
    last_update_time_ = time_->t();
    new_data_ = false;

    // Coordinate systems used:
    // Local level: East/North/Up
    // Body: Nose/Left/Up

    bool oor = false;
    double range = 0.0;
    if (use_flat_earth_ == true) {
        // Get appropriate conversion
        Eigen::Matrix4d tf_m = parent_->state_truth()->tf_matrix(false) *
                               transform()->tf_matrix();
        // Transform sensor's origin to world coordinates
        Eigen::Vector4d sensor_pos = tf_m * Eigen::Vector4d(0, 0, 0, 1);
        Eigen::Vector3d sensor_pos_w = sensor_pos.head<3>() + parent_->state_truth()->pos();

        // Transform ray's end point to world coordinates
        Eigen::Vector4d ray = tf_m * Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
        Eigen::Vector3d ray_w = ray.head<3>();

        // Verify that this will intersect the ground
        if ((sensor_pos_w(2) < 0.0) || (ray_w(2) >= 0.0)) {  // Out of range
            oor = true;
        } else {  // Potentially in range
            // Project the unit vector intersection to the ground.
            range = sensor_pos_w(2) / (-1.0 * ray_w(2));
        }
    } else {
        // Use a service to get range to collision
        // Without a service available, this returns oor at all times
        range = range_;
        oor = oor_;
    }

    if ((oor == false) && (probability_oor_error_ > 0.0)) {
        // Check the random error to see whether it could swap in-range/out-of-range.
        bool swap_in_out_range =
            (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX)) < probability_oor_error_;

        if ((((range < min_range()) || (range > max_range())) && !swap_in_out_range) ||
                (((range > min_range()) && (range < max_range())) && swap_in_out_range)) {
            // Out of range
            oor = true;
        } else {  // In-range
            // Limit to the range boundaries (in case an error caused an in-range
            //  measurement when it was actually out of range)
            range = MIN(MAX(range, min_range()), max_range());
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
            range = MIN(MAX(range + error, min_range()), max_range());
        }

        // Finally, add the actual noise
        double total_sd = range_sd_min_ + (range_sd_per_unit_ * range);
        if (total_sd > 0.0) {
            std::normal_distribution<double> noise_dist(0.0, total_sd);
            range = MIN(MAX(range + noise_dist(generator_), min_range()), max_range());
        }
        // Set the values to the message
        msg->data.set_range(range);
        msg->data.set_range_var(total_sd * total_sd);
    } else {  // Out of range
        // Set the values to the message
        msg->data.set_range(oor_return_);
        msg->data.set_range_var(range_sd_oor_ * range_sd_oor_);
    }

    // Publish the message and return all good
    los_pub_->publish(msg);
    return retVal;
}
}  // namespace sensor
}  // namespace scrimmage
