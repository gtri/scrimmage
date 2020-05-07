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
#include <scrimmage/plugins/autonomy/ROSAirSim/ROSAirSim.h>
#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <rosgraph_msgs/Clock.h>

#include <iostream>
#include <limits>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ROSAirSim, ROSAirSim_plugin)

namespace scrimmage {
namespace autonomy {

ROSAirSim::ROSAirSim() {}

void ROSAirSim::init(std::map<std::string, std::string> &params) {
    show_camera_images_ = scrimmage::get<bool>("show_camera_images", params, "false");
    pub_image_data_ = sc::get<bool>("pub_image_data", params, "true");
    pub_lidar_data_ = sc::get<bool>("pub_lidar_data", params, "true");
    cout << " " << endl;
    if (pub_image_data_) {
        cout << "Publishing AirSim images to ROS." << endl;
    }
    if (pub_lidar_data_) {
        cout << "Publishing AirSim LIDAR data to ROS." << endl;
    }
    if (show_camera_images_) {
        cout << "ROSAirSim: Showing camera images in OpenCV windows." << endl;
    }

    // initialize ros
    if (!ros::isInitialized()) {
        int argc = 0;
        // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_shared<ros::NodeHandle>();

    // Setup robot namespace
    ros_name_ = sc::get<std::string>("ros_namespace_prefix", params, "robot");
    ros_namespace_ = ros_name_ + std::to_string(parent_->id().id());

    // setup image transport node
    // image_transport::ImageTransport it(nh_);
    it_ = std::make_shared<image_transport::ImageTransport>(*nh_);

    // Setup Publishers
    base_scan_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/base_scan", 1);

    // Create TF: Transformation Broadcaster for LIDAR
    //////////////////////////////////////////////////////////////////////
    laser_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    // Create World Transform
    sc::StatePtr &state = parent_->state_truth();
    world_trans_.header.frame_id = "world";
    world_trans_.child_frame_id = ros_namespace_ + "/base_link";
    world_trans_.transform.translation.x = state->pos().x();
    world_trans_.transform.translation.y = state->pos().y();
    world_trans_.transform.translation.z = state->pos().z();
    world_trans_.transform.rotation.x = state->quat().x();
    world_trans_.transform.rotation.y = state->quat().y();
    world_trans_.transform.rotation.z = state->quat().z();
    world_trans_.transform.rotation.w = state->quat().w();
    world_trans_.header.stamp = ros::Time::now();
    // tf_msg_vec_.push_back(world_trans_);
    laser_broadcaster_->sendTransform(world_trans_);

    // Fill in Laser Settings
    // TODO: Pull these values from the settings.json file on the windows side.
    LidarSetting lidar_setting;
    lidar_setting.position.x() = 0.0;
    lidar_setting.position.y() = 0.0;
    lidar_setting.position.z() = 0.0;
    lidar_setting.rotation.roll = M_PI;
    lidar_setting.rotation.pitch = 0.0;
    lidar_setting.rotation.yaw = 0.0;

    laser_trans_.header.frame_id = ros_namespace_ + "/base_link";
    laser_trans_.child_frame_id = ros_namespace_ + "/base_laser";
    laser_trans_.transform.translation.x = lidar_setting.position.x();
    laser_trans_.transform.translation.y = lidar_setting.position.y();
    laser_trans_.transform.translation.z = lidar_setting.position.z();

    tf2::Quaternion laser_quat;
    laser_quat.setRPY(lidar_setting.rotation.roll, lidar_setting.rotation.pitch, lidar_setting.rotation.yaw);
    laser_trans_.transform.rotation.x = laser_quat.x();
    laser_trans_.transform.rotation.y = laser_quat.y();
    laser_trans_.transform.rotation.z = laser_quat.z();
    laser_trans_.transform.rotation.w = laser_quat.w();
    // Send Transform
    laser_trans_.header.stamp = ros::Time::now();
    //  tf_msg_vec_.push_back(laser_trans_);
    // laser_broadcaster_->sendTransform(tf_msg_vec_);
    laser_broadcaster_->sendTransform(laser_trans_);
    //////////////////////////////////////////////////////////////////////

    // airsim lidar callback
    auto airsim_lidar_cb = [&](auto &msg) {
        if (msg->data.lidar_data.point_cloud.size() < 3) { // return if empty message
            return;
        }
        lidar_data_ = msg->data.lidar_data;
    };

    // airsim image callback
    auto airsim_image_cb = [&](auto &msg) {
        if (msg->data.size() == 0) { // return if empty message
            return;
        }
        if (pub_image_data_) {
            // Create the ROSmsg header for the soon to be published messages
            std_msgs::Header header; // empty header
            header.seq = msg->data[0].frame_num; // AirSim defined counter
            header.stamp = ros::Time::now(); // time

            // For each AirSim msg of (camera_config, img) in msg vector
            for (sc::sensor::AirSimImageType a : msg->data) {
                sensor_msgs::ImagePtr img_msg;

                // Create the topic name
                std::string camera_name = boost::algorithm::to_lower_copy(a.camera_config.cam_name);
                std::string image_type_name = boost::algorithm::to_lower_copy(a.camera_config.img_type_name);
                std::string topic_name = "/" + ros_namespace_ + "/" + camera_name + "_" + image_type_name;
                std::string topic_name_pub = ros_namespace_ + "/" + camera_name + "_" + image_type_name;
                // cout << topic_name << endl;
                // cout << "img size:" << a.img.size() << endl;
                // cout << "img channels:" << a.img.channels() << endl;

                // Check if topic publisher exists for each AirSim msg type, if so publish
                bool published = false;
                for (auto pub : img_publishers_) {

                    // If the topic publisher exists, publish to topic
                    if (pub.getTopic() == topic_name) {
                        published = true;
                        if (topic_name == "/robot1/front_center_depthperspective" || topic_name == "/robot1/front_center_depthplanner") {
                            img_msg = cv_bridge::CvImage(header, "", a.img).toImageMsg();
                        } else {
                            img_msg = cv_bridge::CvImage(header, "rgb8", a.img).toImageMsg();
                        }
                        pub.publish(img_msg);
                    }
                }

                // If it gets through the above for loop with published=false then the topic doesn't exist
                if (published == false) {
                    // cout << "Do this once." << endl;
                    // cout << topic_name << endl;

                    // create new publisher for the new topic
                    img_publishers_.push_back(it_->advertise(topic_name_pub, 1));

                    // Publish message so we don't skip frame
                    if (topic_name == "/robot1/front_center_depthperspective" || topic_name == "/robot1/front_center_depthplanner") {
                        img_msg = cv_bridge::CvImage(header, "", a.img).toImageMsg();
                    } else {
                        img_msg = cv_bridge::CvImage(header, "rgb8", a.img).toImageMsg();
                    }
                    // publish using last publisher created
                    img_publishers_.back().publish(img_msg);

                    // topic name should print twice, checking that no weird threading error is occurring
                    // cout << topic_name << endl;
                    // cout << img_publishers_.back().getTopic() << endl;
                }
                // draw image
                if (show_camera_images_) {
                    std::string window_name = a.vehicle_name + "_" + a.camera_config.cam_name + "_" + a.camera_config.img_type_name;
                    if (a.camera_config.img_type_name == "DepthPerspective" || a.camera_config.img_type_name == "DepthPlanner") {
                        cv::Mat tempImage;
                        a.img.convertTo(tempImage, CV_32FC1, 1.f/255);
                        // cv::normalize(a.img, tempImage, 0, 1, cv::NORM_MINMAX);
                        // cout << tempImage << endl;
                        cv::imshow(window_name, tempImage);
                    } else {
                        // other image types are int 0-255.
                        cv::imshow(window_name, a.img);
                    }
                    cv::waitKey(1);
                }
            }
            // TODO: Odom transform broadcaster for camera position on drone.
        }
    };
    if (pub_lidar_data_) {
        subscribe<sensor::AirSimLidarType>("LocalNetwork", "AirSimLidar", airsim_lidar_cb);
    }
    if (pub_image_data_) {
        subscribe<std::vector<sensor::AirSimImageType>>("LocalNetwork", "AirSimImages", airsim_image_cb);
    }

    // Setup Image Topics
}

bool ROSAirSim::step_autonomy(double t, double dt) {
    ros::spinOnce(); // check for new ROS messages

    // Create PointCloud2 message
    //////////////////////////////////////////////////////////////////////
    if (pub_lidar_data_) {
        sensor_msgs::PointCloud2 lidar_msg;
        lidar_msg.header.stamp = ros::Time::now();
        lidar_msg.header.frame_id = ros_namespace_ + "/base_laser";
        if (lidar_data_.point_cloud.size() > 3) {
            lidar_msg.height = 1;
            lidar_msg.width = lidar_data_.point_cloud.size() / 3;

            lidar_msg.fields.resize(3);
            lidar_msg.fields[0].name = "x";
            lidar_msg.fields[1].name = "y";
            lidar_msg.fields[2].name = "z";
            int offset = 0;

            for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
                lidar_msg.fields[d].offset = offset;
                lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
                lidar_msg.fields[d].count  = 1;
            }

            lidar_msg.is_bigendian = false;
            lidar_msg.point_step = offset; // 4 * num fields
            lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

            lidar_msg.is_dense = true;
            std::vector<float> data_std = lidar_data_.point_cloud;

            const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&data_std[0]);
            vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
            lidar_msg.data = std::move(lidar_msg_data);
        }
        base_scan_pub_.publish(lidar_msg);
    }
    //////////////////////////////////////////////////////////////////////
    // Publish Laser Transform
    //////////////////////////////////////////////////////////////////////
    // Update World Transform
    sc::StatePtr &state = parent_->state_truth();
    world_trans_.header.frame_id = "world";
    world_trans_.child_frame_id = ros_namespace_ + "/base_link";
    world_trans_.transform.translation.x = state->pos().x();
    world_trans_.transform.translation.y = state->pos().y();
    world_trans_.transform.translation.z = state->pos().z();
    world_trans_.transform.rotation.x = state->quat().x();
    world_trans_.transform.rotation.y = state->quat().y();
    world_trans_.transform.rotation.z = state->quat().z();
    world_trans_.transform.rotation.w = state->quat().w();
    world_trans_.header.stamp = ros::Time::now();
    // tf_msg_vec_.push_back(world_trans_);
    laser_broadcaster_->sendTransform(world_trans_);

    laser_trans_.header.stamp = ros::Time::now();
    // tf_msg_vec_.push_back(laser_trans_);
    // laser_broadcaster_->sendTransform(tf_msg_vec_);
    laser_broadcaster_->sendTransform(laser_trans_);
    //////////////////////////////////////////////////////////////////////

    return true;
}
} // namespace autonomy
} // namespace scrimmage
