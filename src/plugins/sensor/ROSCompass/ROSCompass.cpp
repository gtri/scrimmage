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
    scrimmage::Angles angles_to_gps;
    sc::StatePtr &state = parent_->state_truth();

    // TODO: Logic and custom message not finished!

    angles_to_gps.set_angle(sc::Angles::rad2deg(state->quat().yaw()));
    // cout << angles_to_gps.angle() << endl;
    double airsim_yaw_rad = sc::Angles::deg2rad(angles_to_gps.angle());
    // cout << airsim_yaw_rad << endl;

    // scrimmage::Angles enu_to_ned_yaw_;
    // enu_to_ned_yaw_.set_angle(ang::rad2deg(state->quat().yaw()));
    // enu_to_ned_yaw_.set_angle(state->quat().yaw());
    // double airsim_yaw_rad = ang::deg2rad(enu_to_ned_yaw_.angle());

    // Fill Compass Message
    sensor_msgs::MagneticField compass_msg;


    // Publish Compass information
    compass_pub_.publish(compass_msg);

    return true;
}
} // namespace sensor
} // namespace scrimmage