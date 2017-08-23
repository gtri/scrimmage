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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <scrimmage/plugins/autonomy/ROSAutonomy/ROSAutonomy.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, ROSAutonomy, ROSAutonomy_plugin)

ROSAutonomy::ROSAutonomy() {}

void ROSAutonomy::init(std::map<std::string,std::string> &params)
{
    if (!ros::isInitialized()) {
        int argc = 0;
        ros::init(argc, NULL, "scrimmage");
    }
    nh_ = std::make_shared<ros::NodeHandle>();

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
    laser_trans_.transform.translation.x = parent_->sensors()["RayTrace0"]->tf()->pos()(0);
    laser_trans_.transform.translation.y = parent_->sensors()["RayTrace0"]->tf()->pos()(1);
    laser_trans_.transform.translation.z = parent_->sensors()["RayTrace0"]->tf()->pos()(2);
    geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(parent_->sensors()["RayTrace0"]->tf()->quat().yaw());
    laser_trans_.transform.rotation = laser_quat;

    pcl_sub_ = create_subscriber(std::to_string(parent_->id().id()) + "/RayTrace0/pointcloud");

    desired_state_->vel() = Eigen::Vector3d::UnitX() * 0;
    desired_state_->quat().set(0,0,state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);
}

bool ROSAutonomy::step_autonomy(double t, double dt)
{
    ros::Time time = ros::Time::now(); // todo

    ros::spinOnce(); // check for new ROS messages

    // Convert scrimmage point cloud into laser scan
    for (auto msg : pcl_sub_->msgs<sc::Message<RayTrace::PointCloud>>()) {
        sensor_msgs::LaserScan laser_msg;
        laser_msg.angle_min = sc::Angles::deg2rad(-20);
        laser_msg.angle_max = sc::Angles::deg2rad(20);
        laser_msg.angle_increment = sc::Angles::deg2rad(2);
        laser_msg.time_increment = 0;
        laser_msg.scan_time = 0;
        laser_msg.range_min = 0.0;
        laser_msg.range_max = 30.0;
        laser_msg.ranges.resize(msg->data.points.size());
        laser_msg.intensities.resize(msg->data.points.size());

        int i = 0;
        for (RayTrace::PCPoint &p : msg->data.points) {
            laser_msg.ranges[i] = p.point.norm();
            laser_msg.intensities[i] = p.intensity;
            i++;
        }
        laser_msg.header.stamp = time;
        laser_msg.header.frame_id = ros_namespace_ + "/base_laser";
        base_scan_pub_.publish(laser_msg);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Publish odometry transform (odom -> base_link)
    odom_trans_.header.stamp = time;
    odom_trans_.transform.translation.x = state_->pos()(0);
    odom_trans_.transform.translation.y = state_->pos()(1);
    odom_trans_.transform.translation.z = state_->pos()(2);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_->quat().yaw());
    odom_trans_.transform.rotation = odom_quat;
    odom_broadcaster_->sendTransform(odom_trans_);

    // Publish odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = ros_namespace_ + "/odom";

    //set the position
    odom.pose.pose.position.x = state_->pos()(0);
    odom.pose.pose.position.y = state_->pos()(1);
    odom.pose.pose.position.z = state_->pos()(2);
    odom.pose.pose.orientation = odom_quat;

    //set the velocity (TODO: Might be wrong frame)
    odom.child_frame_id = ros_namespace_ + "/base_link";
    odom.twist.twist.linear.x = state_->vel()(0);
    odom.twist.twist.linear.y = state_->vel()(1);
    odom.twist.twist.linear.z = state_->vel()(2);
    odom.twist.twist.angular.z = 0; // TODO

    odom_pub_.publish(odom);

    ///////////////////////////////////////////////////////////////////////////
    // Publish odometry transform (base_link -> laser_scan)
    laser_trans_.header.stamp = time;
    laser_broadcaster_->sendTransform(laser_trans_);

    // // Publish odometry message
    // nav_msgs::Odometry odom_base_link;
    // odom_base_link.header.stamp = time;
    // odom_base_link.header.frame_id = "base_link";
    //
    // //set the position
    // odom_base_link.pose.pose.position.x = 0;
    // odom_base_link.pose.pose.position.y = 0;
    // odom_base_link.pose.pose.position.z = 0;
    // odom_base_link.pose.pose.orientation = laser_quat;
    //
    // //set the velocity (TODO: Might be wrong frame)
    // odom_base_link.child_frame_id = "base_laser";
    // odom_base_link.twist.twist.linear.x = 0;
    // odom_base_link.twist.twist.linear.y = 0;
    // odom_base_link.twist.twist.linear.z = 0;
    // odom_base_link.twist.twist.angular.z = 0;
    //
    // odom_laser_pub_.publish(odom_base_link);

    // Update desired state based on "cmd_vel" from ROS
    //desired_state_->vel() = Eigen::Vector3d::UnitX()*1;//cmd_vel_.linear.x;
    //double u_rot = -0.3;//cmd_vel_.angular.z;

    desired_state_->vel()(0) = cmd_vel_.linear.x;
    desired_state_->vel()(1) = cmd_vel_.angular.z;
    desired_state_->vel()(2) = 0;
    //double u_rot = cmd_vel_.angular.z;
    //desired_state_->quat().set(0,0,state_->quat().yaw() + u_rot);
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    return true;
}

void ROSAutonomy::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_ = *msg;
}
