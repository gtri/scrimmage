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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ROSAIRSIM_ROSAIRSIM_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ROSAIRSIM_ROSAIRSIM_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <map>
#include <memory>
#include <vector>

#include "common/AirSimSettings.hpp"

typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;

namespace scrimmage {
namespace autonomy {

class ROSAirSim : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    bool show_camera_images_ = false;
    bool pub_image_data_ = true;
    bool pub_lidar_data_ = true;

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::vector<image_transport::Publisher> img_publishers_;

    ros::Publisher base_scan_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> laser_broadcaster_;
    std::vector<geometry_msgs::TransformStamped> tf_msg_vec_;
    geometry_msgs::TransformStamped laser_trans_;

    std::string ros_namespace_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_ROSAIRSIM_ROSAIRSIM_H_
