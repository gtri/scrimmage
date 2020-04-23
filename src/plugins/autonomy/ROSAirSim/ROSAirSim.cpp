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

    // airsim lidar callback
    auto airsim_lidar_cb = [&](auto &msg) {
        if (msg->data.lidar_data.point_cloud.size() < 3) { // return if empty message
            return;
        }
        lidar_data_ = msg->data;
    };

    // airsim image callback
    auto airsim_image_cb = [&](auto &msg) {
        image_data_ = msg->data;
        if (image_data_.size() == 0) { // return if empty message
            return;
        }

        img_topic_published_mutex_.lock();
        bool img_topic_published = img_topic_published_;
        img_topic_published_mutex_.unlock();

        // If topics are not published create them
        if(!img_topic_published) {
            // for each image in the message
            for (sc::sensor::AirSimImageType a : image_data_) {

                // Create the topic name
                std::string camera_name = boost::algorithm::to_lower_copy(a.camera_config.cam_name);
                std::string image_type_name = boost::algorithm::to_lower_copy(a.camera_config.img_type_name);
                std::string topic_name = "/" + ros_namespace_ + "/" + camera_name + "/" + image_type_name;
                img_publishers_.push_back(it_->advertise(topic_name, 1));

                // Publish current message so we don't miss any images
                std_msgs::Header header; // empty header
                header.stamp = ros::Time::now(); // time
                header.frame_id = ros_namespace_ + "/" + camera_name + "/images";
                sensor_msgs::ImagePtr img_msg;
                // Depth Images (Depth Perspective and Depth Planner) come in as 1 channel float arrays
                if (a.camera_config.pixels_as_float) {
                    img_msg = cv_bridge::CvImage(header, "", a.img).toImageMsg();
                } else {
                    img_msg = cv_bridge::CvImage(header, "rgb8", a.img).toImageMsg();
                }
                // Publish to the last publisher added to img_publishers_
                img_publishers_.back().publish(img_msg);

                //// publish a transform for each unique camera_name
                // Note down camera names since each camera will need its own image transform
                // old_cam_name=true if camera_name already exists
                bool old_cam_name = false;
                old_cam_name = std::any_of(camera_names_.begin(), camera_names_.end(), [camera_name](std::string str){return str==camera_name;});
                // If still false, this is a new camera name, save and publish transform
                if(!old_cam_name){
                    // cout << "New camera found: " << camera_name << endl;
                    camera_names_.push_back(camera_name);

                    // cout << "publishing transform for: " << camera_name << endl;
                    // Publish Image transform to broadcaster
                    geometry_msgs::TransformStamped image_trans_;
                    image_trans_.header.frame_id = ros_namespace_ + "/base_link";
                    image_trans_.child_frame_id = ros_namespace_ + "/" + camera_name + "/pose";
                    image_trans_.header.stamp = ros::Time::now();

                    // Pose in image_data.pose is in NED, but scrimmage is in ENU so use the conversion done in AirSimSensor
                    // Position
                    image_trans_.transform.translation.x = a.camera_config.cam_position_ENU.x();
                    image_trans_.transform.translation.y = a.camera_config.cam_position_ENU.y();
                    image_trans_.transform.translation.z = a.camera_config.cam_position_ENU.z();
                    // Orientation
                    image_trans_.transform.rotation.w = a.camera_config.cam_orientation_ENU.w();
                    image_trans_.transform.rotation.x = a.camera_config.cam_orientation_ENU.x();
                    image_trans_.transform.rotation.y = a.camera_config.cam_orientation_ENU.y();
                    image_trans_.transform.rotation.z = a.camera_config.cam_orientation_ENU.z();
                    laser_broadcaster_->sendTransform(image_trans_);
                } // end if new camera name to publish pose for

            } // end for loop a : images in msg

            img_topic_published_mutex_.lock();
            img_topic_published_ = true;
            img_topic_published_mutex_.unlock();

        } // end if img_topic_published=false
    };

    if (pub_lidar_data_) {
        subscribe<sensor::AirSimLidarType>("LocalNetwork", "AirSimLidar", airsim_lidar_cb);
    }
    if (pub_image_data_) {
        subscribe<std::vector<sensor::AirSimImageType>>("LocalNetwork", "AirSimImages", airsim_image_cb);
    }
}

bool ROSAirSim::step_autonomy(double t, double dt) {
    ros::spinOnce(); // check for new ROS messages
    std::vector<geometry_msgs::TransformStamped> tf_msg_vec_;
    tf_msg_vec_.clear();

    // Create LIDAR PointCloud2 ROS message
    //////////////////////////////////////////////////////////////////////
    if (pub_lidar_data_) {
        sensor_msgs::PointCloud2 lidar_msg;
        lidar_msg.header.stamp = ros::Time::now();
        lidar_msg.header.frame_id = ros_namespace_ + "/base_laser";
        if (lidar_data_.lidar_data.point_cloud.size() > 3) {
            lidar_msg.height = 1;
            lidar_msg.width = lidar_data_.lidar_data.point_cloud.size() / 3;

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
            std::vector<float> data_std = lidar_data_.lidar_data.point_cloud;

            const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&data_std[0]);
            vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
            lidar_msg.data = std::move(lidar_msg_data);
        }
        base_scan_pub_.publish(lidar_msg);

        //////////////////////////////////////////////////////////////////////
        // Publish Laser Transform
        //////////////////////////////////////////////////////////////////////
        geometry_msgs::TransformStamped laser_trans_;
        laser_trans_.header.frame_id = ros_namespace_ + "/base_link";
        laser_trans_.child_frame_id = ros_namespace_ + "/base_laser";
        laser_trans_.header.stamp = ros::Time::now();

        // Pose in lidar_data.pose is in NED, but scrimmage is in ENU so use the conversion done in AirSimSensor
        // Position
        laser_trans_.transform.translation.x = lidar_data_.lidar_position_ENU.x();
        laser_trans_.transform.translation.y = lidar_data_.lidar_position_ENU.y();
        laser_trans_.transform.translation.z = lidar_data_.lidar_position_ENU.z();
        // Orientation
        laser_trans_.transform.rotation.w = lidar_data_.lidar_orientation_ENU.w();
        laser_trans_.transform.rotation.x = lidar_data_.lidar_orientation_ENU.x();
        laser_trans_.transform.rotation.y = lidar_data_.lidar_orientation_ENU.y();
        laser_trans_.transform.rotation.z = lidar_data_.lidar_orientation_ENU.z();
        tf_msg_vec_.push_back(laser_trans_);
        // laser_broadcaster_->sendTransform(tf_msg_vec_);
        // laser_broadcaster_->sendTransform(laser_trans_);
    }

    // Update Images
    //////////////////////////////////////////////////////////////////////
    img_topic_published_mutex_.lock();
    bool img_topic_published = img_topic_published_;
    img_topic_published_mutex_.unlock();
    if (pub_image_data_ && img_topic_published) {
            // All image topics should be published
        for (sc::sensor::AirSimImageType a : image_data_) {

            // create string to match using topic name
            std::string camera_name = boost::algorithm::to_lower_copy(a.camera_config.cam_name);
            std::string image_type_name = boost::algorithm::to_lower_copy(a.camera_config.img_type_name);
            std::string topic_name_match = "/" + ros_namespace_ + "/" + camera_name + "/" + image_type_name;
            std_msgs::Header header; // empty header
            header.stamp = ros::Time::now(); // time
            header.frame_id = ros_namespace_ + "/" + camera_name + "/images";
            sensor_msgs::ImagePtr img_msg;

            // cout << "img_publishers_ length: " << img_publishers_.size() << endl;
            for (auto pub : img_publishers_) {
                // If the topic publisher exists, publish to topic
                if (pub.getTopic() == topic_name_match) {
                    // Depth Images (Depth Perspective and Depth Planner) come in as 1 channel float arrays
                    if (a.camera_config.pixels_as_float) {
                        img_msg = cv_bridge::CvImage(header, "", a.img).toImageMsg();
                    } else {
                        img_msg = cv_bridge::CvImage(header, "rgb8", a.img).toImageMsg();
                    }
                    pub.publish(img_msg);
                    break; // break publish for loop after image is published
                } // end if topic name matches
            } // end publishers for loop

            // draw image
            if (show_camera_images_) {
                std::string window_name = a.vehicle_name + "_" + a.camera_config.cam_name + "_" + a.camera_config.img_type_name;
                if (a.camera_config.pixels_as_float) {
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
            } // end draw image
        } // end images in message for loop


        // for each camera_name publish a transform
        // cout << "camera_names length: " << camera_names_.size() << endl;
        for (std::string cam_name : camera_names_) {
            for (sc::sensor::AirSimImageType a : image_data_) {
                if (a.camera_config.cam_name == cam_name) {
                    // cout << "publishing transform for: " << cam_name << endl;
                    // create and publish camera_name transform from image in msg
                    geometry_msgs::TransformStamped image_trans_;
                    image_trans_.header.frame_id = ros_namespace_ + "/base_link";
                    image_trans_.child_frame_id = ros_namespace_ + "/" + cam_name + "/pose";
                    image_trans_.header.stamp = ros::Time::now();

                    // Pose in image_data.pose is in NED, but scrimmage is in ENU so use the conversion done in AirSimSensor
                    // Position
                    image_trans_.transform.translation.x = a.camera_config.cam_position_ENU.x();
                    image_trans_.transform.translation.y = a.camera_config.cam_position_ENU.y();
                    image_trans_.transform.translation.z = a.camera_config.cam_position_ENU.z();
                    // Orientation
                    image_trans_.transform.rotation.w = a.camera_config.cam_orientation_ENU.w();
                    image_trans_.transform.rotation.x = a.camera_config.cam_orientation_ENU.x();
                    image_trans_.transform.rotation.y = a.camera_config.cam_orientation_ENU.y();
                    image_trans_.transform.rotation.z = a.camera_config.cam_orientation_ENU.z();
                    tf_msg_vec_.push_back(image_trans_);
                    // laser_broadcaster_->sendTransform(image_trans_);
                    break; // break the image in msg for loop if camera_name is found and move to next camera_name
                }
            }
        }
    }
    //////////////////////////////////////////////////////////////////////
    // Update World Transform
    //////////////////////////////////////////////////////////////////////
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
    tf_msg_vec_.push_back(world_trans_);
    laser_broadcaster_->sendTransform(tf_msg_vec_);

//    world_trans_.header.frame_id = "world";
//    world_trans_.child_frame_id = ros_namespace_ + "/base_link";
//    world_trans_.transform.translation.x = lidar_data_.vehicle_pose.translation().x();
//    world_trans_.transform.translation.y = lidar_data_.vehicle_pose.translation().y();
//    world_trans_.transform.translation.z = lidar_data_.vehicle_pose.translation().z();
//    Eigen::Quaternionf tf_vehicle_pose_rotation(lidar_data_.vehicle_pose.rotation());
//    world_trans_.transform.rotation.x = tf_vehicle_pose_rotation.x();
//    world_trans_.transform.rotation.y = tf_vehicle_pose_rotation.y();
//    world_trans_.transform.rotation.z = tf_vehicle_pose_rotation.z();
//    world_trans_.transform.rotation.w = tf_vehicle_pose_rotation.w();
//    world_trans_.header.stamp = ros::Time::now();
//    tf_msg_vec_.push_back(world_trans_);
//    laser_broadcaster_->sendTransform(tf_msg_vec_);
    //////////////////////////////////////////////////////////////////////

    return true;
}
} // namespace autonomy
} // namespace scrimmage
