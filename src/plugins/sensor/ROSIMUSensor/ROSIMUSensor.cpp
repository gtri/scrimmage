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
#include <math.h>
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

#include <iostream>
#include <fstream>

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

    // Open imu_data CSV for append (app) and set column headers
    std::string csv_filename = parent_->mp()->log_dir() + "/imu_data.csv";
    if (!csv.open_output(csv_filename, std::ios_base::app)) std::cout << "Couldn't create csv file" << endl;
    if (!csv.output_is_open()) cout << "File isn't open. Can't write to CSV" << endl;
    csv.set_column_headers("time, dt, ENU_POSX, ENU_POSY, ENU_POSZ, ENU_VELX, ENU_VELY, ENU_VELZ, BodytoNED_roll, BodytoNED_pitch, BodytoNED_yaw");

    prev_vel_ = parent_->state_truth()->vel();
    prev_quat_ = parent_->state_truth()->quat();
    prev_time_ = time_->t();
}

bool ROSIMUSensor::step() {
    // Obtain current state information
    sc::StatePtr &state = parent_->state_truth();
    double time_now = time_->t();
    double dt = time_now - prev_time_;
    // cout << " " << endl;
    // cout << "Current Vel: \n" << state->vel() << endl;
    // cout << "Prev Vel: \n" << prev_vel_ << endl;
    // cout << "dt: " << dt << endl;

    // Fill IMU Message
    sensor_msgs::Imu imu_msg;

    // Header
    std_msgs::Header header; // empty header
    // TODO: header.frame = ? // No system for ROS Frame ID's yet
    header.stamp = ros::Time::now(); // time
    imu_msg.header = header;

    // Orientation
    imu_msg.orientation.x = state->quat().x();
    imu_msg.orientation.y = state->quat().y();
    imu_msg.orientation.z = state->quat().z();
    imu_msg.orientation.w = state->quat().w();

    // Angular Velocity
    // cout << "D Theta X: " << (state->quat().roll() - prev_quat_.roll()) / dt << endl;
    imu_msg.angular_velocity.x = (state->quat().roll() - prev_quat_.roll()) / dt;
    imu_msg.angular_velocity.y = (state->quat().pitch() - prev_quat_.pitch()) / dt;
    imu_msg.angular_velocity.z = (state->quat().yaw() - prev_quat_.yaw()) / dt;

    // Linear Acceleration
    // cout << "D Vel X: " << (state->vel().x() - prev_vel_.x()) / dt << endl;
    imu_msg.linear_acceleration.x = (state->vel().x() - prev_vel_.x()) / dt;
    imu_msg.linear_acceleration.y = (state->vel().y() - prev_vel_.y()) / dt;
    imu_msg.linear_acceleration.z = (state->vel().z() - prev_vel_.z()) / dt;

    // Publish Compass information
    imu_pub_.publish(imu_msg);

    // Update prev_state_
    prev_vel_ = state->vel();
    prev_quat_ = state->quat();
    prev_time_ = time_now;

    // Convert RPY_ENUtoBody to RPYBodytoNED
    // ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
    // Convert ENUtoBody to NED
    // Angles to rotate by
    double zTo = 90 * (M_PI / 180); // +PI/2 rotation about Z (Up)
    double yTo = 180 * (M_PI / 180); // +PI rotation about X (Y = new X after Z rotation)
    // Normalize entity quaternion
    Eigen::Quaterniond ENU_quat = state->quat();
    // Testing with different initial quaternions
    // Eigen::Quaterniond ENU_quat = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0); // RPY init = 0,0,0 final = 180,0,90
    // Eigen::Quaterniond ENU_quat = Eigen::Quaterniond(0.0, 0.7071068, -0.7071068, 0.0); // RPY init = 180,0,90 final = 0,0,180
    ENU_quat.normalize();
    // cout << " " << endl;
    // cout << "ENUtoBody_quat: \n" << ENU_quat.w() << "\n" <<  ENU_quat.vec() << endl;

    // Rotate ENU Quaternion to NED, Order matters
    Eigen::Quaterniond NED_quat;
    NED_quat = Eigen::AngleAxisd(zTo, Eigen::Vector3d::UnitZ()) * ENU_quat; // +PI/2 rotation about Z (Up)
    NED_quat = Eigen::AngleAxisd(yTo, Eigen::Vector3d::UnitY()) * NED_quat; // +PI rotation about new X (Y = new X after Z rotation)

    // Take Inverse
    Eigen::Quaterniond BodytoNED_quat = NED_quat.inverse();
    // cout << "BodytoNED_quat: \n" << BodytoNED_quat.w() << "\n" << BodytoNED_quat.vec() << endl;

    // Get RPY
    Quaternion ned;
    ned.set(BodytoNED_quat.w(),
            BodytoNED_quat.vec().x(),
            BodytoNED_quat.vec().y(),
            BodytoNED_quat.vec().z());
    // cout << "BodytoNED RPY (rad): " << ned.roll() << " " << ned.pitch() << " " << ned.yaw() << endl;
    // cout << "BodytoNED RPY (deg): " << ned.roll()*(180 / M_PI) << " " << ned.pitch()*(180 / M_PI) << " " << ned.yaw()*(180 / M_PI) << endl;

    // cout << " " << endl;
    // cout << " " << endl;
    // cout << " " << endl;
    // cout << " " << endl;
    // Write IMU data to CSV
    // Write the CSV file to the root log directory file name = imu_data.csv
    if (!csv.output_is_open()) {
            cout << "File isn't open. Can't append to CSV" << endl;
            }
    csv.append(sc::CSV::Pairs{
    {"time", time_now},
    {"dt", dt},
    {"ENU_POSX", state->pos()(0)},
    {"ENU_POSY", state->pos()(1)},
    {"ENU_POSZ", state->pos()(2)},
    {"ENU_VELX", state->vel()(0)},
    {"ENU_VELY", state->vel()(1)},
    {"ENU_VELZ", state->vel()(2)},
    {"BodytoNED_roll", ned.roll()},
    {"BodytoNED_pitch", ned.pitch()},
    {"BodytoNED_yaw", ned.yaw()}}, true, true);

    return true;
}

void ROSIMUSensor::close(double t) {
    csv.close_output();
}
} // namespace sensor
} // namespace scrimmage
