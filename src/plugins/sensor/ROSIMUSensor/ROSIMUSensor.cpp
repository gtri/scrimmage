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

#include <scrimmage/plugins/sensor/ROSIMUSensor/ROSIMUSensor.h>

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

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::ROSIMUSensor, ROSIMUSensor_plugin)

namespace scrimmage {
namespace sensor {

ROSIMUSensor::ROSIMUSensor() {}

void ROSIMUSensor::init(std::map<std::string, std::string> &params) {

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
    imu_pub_ = nh_->advertise<sensor_msgs::Imu>(ros_namespace_ + "/imu", 1);

    state_6dof_ = std::make_shared<motion::RigidBody6DOFState>();
    auto cb = [&](auto &msg){*state_6dof_ = msg->data;};
    subscribe<motion::RigidBody6DOFState>("LocalNetwork", "RigidBody6DOFState", cb);
}

bool ROSIMUSensor::step() {
    // Obtain current state information
    sc::StatePtr &state = parent_->state_truth();
    // cout << *state_6dof_ << endl;
    // cout << state_6dof_->linear_accel_body()(0) << endl;
    // cout << state_6dof_->linear_accel_body()(1) << endl;
    // cout << state_6dof_->linear_accel_body()(2) << endl;

    // Fill Compass Message
    sensor_msgs::Imu imu_msg;

    // Header
    std_msgs::Header header; // empty header
    // TODO: header.frame = ? // No system for ROS Frame ID's yet
    header.stamp = ros::Time::now(); // time
    imu_msg.header = header;

    // Orientation
    imu_msg.orientation.x = state->pos()(0);
    imu_msg.orientation.y = state->pos()(1);
    imu_msg.orientation.z = state->pos()(2);
    imu_msg.orientation.w = 1.0;

    // Angular Velocity
    imu_msg.angular_velocity.x = state_6dof_->ang_vel_body()(0);
    imu_msg.angular_velocity.y = -state_6dof_->ang_vel_body()(1);
    imu_msg.angular_velocity.z = -state_6dof_->ang_vel_body()(2);

    // Linear Acceleration
    imu_msg.linear_acceleration.x = state_6dof_->linear_accel_body()(0);
    imu_msg.linear_acceleration.y = -state_6dof_->linear_accel_body()(1);
    imu_msg.linear_acceleration.z = -state_6dof_->linear_accel_body()(2);

    // Publish Compass information
    imu_pub_.publish(imu_msg);

    return true;
}
} // namespace sensor
} // namespace scrimmage
