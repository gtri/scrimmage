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


#include <rosgraph_msgs/Clock.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <scrimmage/plugins/autonomy/ROSAutonomy/ROSAutonomy.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>

#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ROSAutonomy, ROSAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

ROSAutonomy::ROSAutonomy() {}

void ROSAutonomy::publish_clock_msg(double t) {
    ros::Time time(t); // start at zero seconds
    rosgraph_msgs::Clock clock_msg; // Message to hold time
    clock_msg.clock = time;
    clock_pub_.publish(clock_msg);
}

void ROSAutonomy::init(std::map<std::string, std::string> &params) {
    speed_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::Out);
    turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);
    velocity_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);

    if (!ros::isInitialized()) {
        int argc = 0;
        // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_shared<ros::NodeHandle>();

    // Setup clock server
    clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 1);
    publish_clock_msg(0);

    // Setup robot namespace
    ros_namespace_ = sc::get<std::string>("ros_namespace_prefix", params, "robot");
    ros_namespace_ += std::to_string(parent_->id().id());

    cmd_vel_sub_ = nh_->subscribe(ros_namespace_ + "/cmd_vel", 1, &ROSAutonomy::cmd_vel_cb, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(ros_namespace_ + "/odom", 1);
    base_scan_pub_ = nh_->advertise<sensor_msgs::LaserScan>(ros_namespace_ + "/base_scan", 1);
    base_pose_truth_pub = nh_->advertise<nav_msgs::Odometry>(ros_namespace_ + "/base_pose_ground_truth", 1);

    odom_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
    laser_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

    odom_trans_.header.frame_id = ros_namespace_ + "/odom";
    odom_trans_.child_frame_id = ros_namespace_ + "/base_link";

    laser_trans_.header.frame_id = ros_namespace_ + "/base_link";
    laser_trans_.child_frame_id = ros_namespace_ + "/base_laser";
    laser_trans_.transform.translation.x = parent_->sensors()["RayTrace0"]->transform()->pos()(0);
    laser_trans_.transform.translation.y = parent_->sensors()["RayTrace0"]->transform()->pos()(1);
    laser_trans_.transform.translation.z = parent_->sensors()["RayTrace0"]->transform()->pos()(2);
    geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(parent_->sensors()["RayTrace0"]->transform()->quat().yaw());
    laser_trans_.transform.rotation = laser_quat;

    auto pc_cb = [&] (scrimmage::MessagePtr<sc::sensor::RayTrace::PointCloud> msg) {
        pcl_ = msg->data;
    };
    subscribe<sc::sensor::RayTrace::PointCloud>("LocalNetwork", "RayTrace/pointcloud", pc_cb);

    vars_.output(speed_idx_, 0);
    vars_.output(turn_rate_idx_, 0);
    vars_.output(pitch_rate_idx_, 0);
    vars_.output(velocity_z_idx_, 0);
}

bool ROSAutonomy::step_autonomy(double t, double dt) {
    // Update ROS time
    publish_clock_msg(t);
    ros::Time ros_time(t);

    ros::spinOnce(); // check for new ROS messages

    // Convert scrimmage point cloud into ROS laser scan message
    // double fov_half = pcl_.num_rays_horiz * pcl_.angle_res_horiz / 2.0;
    sensor_msgs::LaserScan laser_msg;
    // laser_msg.angle_min = -fov_half;
    // laser_msg.angle_max = fov_half;
    // laser_msg.angle_increment = pcl_.angle_res_horiz;
    laser_msg.time_increment = 0;
    laser_msg.scan_time = 0;
    laser_msg.range_min = pcl_.min_range;
    laser_msg.range_max = pcl_.max_range;
    laser_msg.ranges.resize(pcl_.points.size());
    laser_msg.intensities.resize(pcl_.points.size());

    int i = 0;
    for (sc::sensor::RayTrace::PCPoint &p : pcl_.points) {
        laser_msg.ranges[i] = p.point.norm();
        laser_msg.intensities[i] = p.intensity;
        i++;
    }
    laser_msg.header.stamp = ros_time;
    laser_msg.header.frame_id = ros_namespace_ + "/base_laser";
    base_scan_pub_.publish(laser_msg);

    ///////////////////////////////////////////////////////////////////////////
    // Publish odometry transform (odom -> base_link)
    odom_trans_.header.stamp = ros_time;
    odom_trans_.transform.translation.x = state_->pos()(0);
    odom_trans_.transform.translation.y = state_->pos()(1);
    odom_trans_.transform.translation.z = state_->pos()(2);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_->quat().yaw());
    odom_trans_.transform.rotation = odom_quat;
    odom_broadcaster_->sendTransform(odom_trans_);

    // Publish odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = ros_time;
    odom.header.frame_id = ros_namespace_ + "/odom";

    // set the position
    odom.pose.pose.position.x = state_->pos()(0);
    odom.pose.pose.position.y = state_->pos()(1);
    odom.pose.pose.position.z = state_->pos()(2);
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = ros_namespace_ + "/base_link";
    odom.twist.twist.linear.x = state_->vel()(0);
    odom.twist.twist.linear.y = state_->vel()(1);
    odom.twist.twist.linear.z = state_->vel()(2);
    odom.twist.twist.angular.z = 0; // TODO

    odom_pub_.publish(odom);

    ///////////////////////////////////////////////////////////////////////////
    // Publish odometry transform (base_link -> laser_scan)
    laser_trans_.header.stamp = ros_time;
    laser_broadcaster_->sendTransform(laser_trans_);

    // Send commands to low-level controller
    vars_.output(speed_idx_, cmd_vel_.linear.x);
    vars_.output(turn_rate_idx_, cmd_vel_.angular.z);
    vars_.output(pitch_rate_idx_, 0);
    vars_.output(velocity_z_idx_, 0);

    return true;
}

void ROSAutonomy::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_ = *msg;
}

} // namespace autonomy
} // namespace scrimmage
