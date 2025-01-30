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
 * @date 06 May 2020
 * @version 0.2.8
 * @brief Receives AirSim data as SCRIMMAGE messages and publishes them as ROS messages.
 * @section Receives AirSim data as SCRIMMAGE messages and publishes them as ROS messages.
 * Receives AirSim data as SCRIMMAGE messages and publishes them as ROS messages.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ROSAIRSIM_ROSAIRSIM_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ROSAIRSIM_ROSAIRSIM_H_
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

// Eigen libraries
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "common/AirSimSettings.hpp"
typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;

namespace scrimmage {
namespace autonomy {

class ROSAirSim : public scrimmage::Autonomy {
 public:
    ROSAirSim();
    Eigen::Isometry3f get_vehicle_world_pose_from_NED_to_ENU(
        Eigen::Isometry3f vehicle_pose_world_NED);
    Eigen::Isometry3f get_sensor_pose_from_worldNED_to_vehicleENU(
        Eigen::Isometry3f vehicle_position_NED, Eigen::Isometry3f sensor_position_NED);
    void init(std::map<std::string, std::string>& params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    std::string vehicle_name_ = "none";
    bool show_camera_images_ = false;
    bool pub_image_data_ = true;
    bool pub_lidar_data_ = true;
    bool pub_imu_data_ = true;
    bool ros_python_ = false;
    bool ros_cartographer_ = false;

    // ROS
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;

    // Images
    vector<scrimmage::sensor::AirSimImageType> image_data_;
    std::vector<image_transport::Publisher> trans_img_publishers_;
    std::vector<ros::Publisher> img_publishers_;
    std::vector<ros::Publisher> cam_info_publishers_;
    bool img_topic_published_ = false;
    std::mutex img_topic_published_mutex_;
    std::vector<std::string> camera_names_;

    // Lidar
    vector<scrimmage::sensor::AirSimLidarType> lidar_data_;
    std::vector<ros::Publisher> lidar_publishers_;
    bool lidar_topic_published_ = false;
    std::mutex lidar_topic_published_mutex_;
    std::vector<std::string> lidar_names_;

    // IMU
    vector<scrimmage::sensor::AirSimImuType> imu_data_;
    std::vector<ros::Publisher> imu_publishers_;
    bool imu_topic_published_ = false;
    std::mutex imu_topic_published_mutex_;
    std::vector<std::string> imu_names_;

    ros::Publisher base_scan_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> laser_broadcaster_;
    static std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    geometry_msgs::TransformStamped world_trans_;
    geometry_msgs::TransformStamped vehicle_trans_;

    std::string ros_name_;
    std::string ros_namespace_;
};
}  // namespace autonomy
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ROSAIRSIM_ROSAIRSIM_H_
