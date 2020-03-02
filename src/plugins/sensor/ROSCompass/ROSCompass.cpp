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

#include <scrimmage/plugins/sensor/ROSCompass/ROSCompass.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/math/Angles.h>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::ROSCompass, ROSCompass_plugin)

namespace scrimmage {
namespace sensor {

ROSCompass::ROSCompass() {}

void ROSCompass::init(std::map<std::string, std::string> &params) {

    if (!ros::isInitialized()) {
        int argc = 0;
        // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_shared<ros::NodeHandle>();

    // Setup robot namespace
    ros_namespace_ = sc::get<std::string>("ros_namespace_prefix", params, "robot");
    ros_namespace_ += std::to_string(parent_->id().id());

    // Create Publisher
    compass_pub_ = nh_->advertise<sensor_msgs::MagneticField>(ros_namespace_ + "/compass", 1);
}

bool ROSCompass::step() {
    // Obtain current state information
    sc::StatePtr &state = parent_->state_truth();

    // Scrimmage is in ENU so all outputs are in ENU (East North Up).

    // Get rotation vector
    // Rotate Reverse: Get rotation between orientation of the body and ENU orientation of the north pole (0, 1, 0)
    Eigen::Vector3d rotation = state->quat().rotate_reverse(Eigen::Vector3d(0.0, 1.0, 0.0));

    // Fill Compass Message. We're using the magnetic field message, but the info is the rotation vector.
    sensor_msgs::MagneticField compass_msg;
    compass_msg.magnetic_field.x = rotation.x();
    compass_msg.magnetic_field.y = rotation.y();
    compass_msg.magnetic_field.z = rotation.z();

    // Header
    std_msgs::Header header; // empty header
    // TODO: header.frame = ? // No system for ROS Frame ID's yet
    header.stamp = ros::Time::now(); // time
    compass_msg.header = header;

    // Publish Compass information
    compass_pub_.publish(compass_msg);

    return true;
}
} // namespace sensor
} // namespace scrimmage
