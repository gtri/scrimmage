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
 * @brief Receives AirSim data as SCRIMMAGE messages and publishes them as ROS
 * messages.
 * @section Receives AirSim data as SCRIMMAGE messages and publishes them as ROS
 * messages. Receives AirSim data as SCRIMMAGE messages and publishes them as
 * ROS messages.
 *
 */
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/ROSAirSim/ROSAirSim.h>
#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>

#include <rosgraph_msgs/Clock.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/string.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ROSAirSim, ROSAirSim_plugin)

namespace scrimmage {
namespace autonomy {

ROSAirSim::ROSAirSim() {}

Eigen::Isometry3f ROSAirSim::get_vehicle_world_pose_from_NED_to_ENU(Eigen::Isometry3f vehicle_pose_world_NED) {
    // Get Vehicle Pose from NED to ENU
    Eigen::Matrix<float, 3, 1> vehicle_position_ENU;
    vehicle_position_ENU.x() = vehicle_pose_world_NED.translation().y();
    vehicle_position_ENU.y() = vehicle_pose_world_NED.translation().x();
    vehicle_position_ENU.z() = -1 * vehicle_pose_world_NED.translation().z();
    // Convert the orientation to ENU
    // Eigen::Quaternion<float> NED_quat_vehicle =
    // vehicle_pose_world_NED.rotation();
    Eigen::Quaternionf NED_quat_vehicle(vehicle_pose_world_NED.rotation());

    // Rotate, order matters
    double xTo = -180 * (M_PI / 180);  // -PI rotation about X
    double zTo = 90 * (M_PI / 180);    // PI/2 rotation about Z (Up)
    Eigen::Quaternion<float> vehicle_orientation_ENU;
    vehicle_orientation_ENU =
        Eigen::AngleAxis<float>(xTo, Eigen::Vector3f::UnitX()) * NED_quat_vehicle;  // -PI rotation about X
    vehicle_orientation_ENU =
        Eigen::AngleAxis<float>(zTo, Eigen::Vector3f::UnitZ()) * vehicle_orientation_ENU;  // PI/2 rotation about Z (Up)
    // Bring vehicle pose in relation to ENU, World into Eigen
    Eigen::Translation3f translation_trans_vehicle(vehicle_position_ENU);
    Eigen::Quaternionf rotation_quat_vehicle(vehicle_orientation_ENU);
    Eigen::Isometry3f tf_world_vehicle_ENU(translation_trans_vehicle * rotation_quat_vehicle);
    return tf_world_vehicle_ENU;
}

Eigen::Isometry3f ROSAirSim::get_sensor_pose_from_worldNED_to_vehicleENU(Eigen::Isometry3f vehicle_pose_world_NED,
                                                                         Eigen::Isometry3f sensor_pose_world_NED) {
    // Get Vehicle Pose from NED to ENU
    Eigen::Isometry3f tf_world_vehicle_ENU = get_vehicle_world_pose_from_NED_to_ENU(vehicle_pose_world_NED);

    // AirSim gives pose of camera in relation to the world frame
    // Pose in response is in NED, but scrimmage is in ENU so convert
    // Convert position to ENU: Switch X and Y and negate Z
    Eigen::Matrix<float, 3, 1> sensor_position_world_ENU;
    sensor_position_world_ENU.x() = sensor_pose_world_NED.translation().y();
    sensor_position_world_ENU.y() = sensor_pose_world_NED.translation().x();
    sensor_position_world_ENU.z() = -1 * sensor_pose_world_NED.translation().z();
    // Convert the orientation to ENU
    // Eigen::Quaternion<float> NED_quat = sensor_pose_world_NED.rotation();
    Eigen::Quaternionf NED_quat(sensor_pose_world_NED.rotation());
    // Rotate, order matters
    double xTo = -180 * (M_PI / 180);  // -PI rotation about X
    double zTo = 90 * (M_PI / 180);    // PI/2 rotation about Z (Up)
    Eigen::Quaternion<float> sensor_orientation_world_ENU;
    sensor_orientation_world_ENU =
        Eigen::AngleAxis<float>(xTo, Eigen::Vector3f::UnitX()) * NED_quat;  // -PI rotation about X
    sensor_orientation_world_ENU = Eigen::AngleAxis<float>(zTo, Eigen::Vector3f::UnitZ()) *
                                   sensor_orientation_world_ENU;  // PI/2 rotation about Z (Up)
    // Place pose of camera in ENU, world frame into Eigen
    Eigen::Translation3f translation_trans_sensor(sensor_position_world_ENU);
    Eigen::Quaternionf rotation_quat_sensor(sensor_orientation_world_ENU);
    Eigen::Isometry3f tf_world_sensor_ENU(translation_trans_sensor * rotation_quat_sensor);

    // Get pose of lidar in ENU, vehicle frame using Eigen
    Eigen::Isometry3f tf_vehicle_sensor_ENU(tf_world_vehicle_ENU.inverse() * tf_world_sensor_ENU);
    return tf_vehicle_sensor_ENU;
}

void ROSAirSim::init(std::map<std::string, std::string> &params) {
    vehicle_name_ = sc::get<std::string>("vehicle_name", params, "robot1");
    show_camera_images_ = scrimmage::get<bool>("show_camera_images", params, "false");
    pub_image_data_ = sc::get<bool>("pub_image_data", params, "true");
    pub_lidar_data_ = sc::get<bool>("pub_lidar_data", params, "true");
    pub_imu_data_ = sc::get<bool>("pub_imu_data", params, "true");
    ros_python_ = sc::get<bool>("ros_python", params, "false");
    ros_cartographer_ = sc::get<bool>("ros_cartographer", params, "false");
    cout << " " << endl;
    cout << "[ROSAirSim] Vehicle Name: " << vehicle_name_ << endl;
    if (pub_image_data_) {
        cout << "[ROSAirSim] Publishing AirSim images to ROS." << endl;
    }
    if (pub_lidar_data_) {
        cout << "[ROSAirSim] Publishing AirSim LIDAR data to ROS." << endl;
    }
    if (pub_imu_data_) {
        cout << "[ROSAirSim] Publishing AirSim IMU data to ROS." << endl;
    }

    if (show_camera_images_) {
        cout << "[ROSAirSim] Showing camera images in OpenCV windows." << endl;
    }
    if (ros_python_) {
        cout << "[ROSAirSim] Using image message for ROS Python API." << endl;
    } else {
        cout << "[ROSAirSim] Using Image Transports ROS message" << endl;
    }
    if (ros_cartographer_) {
        cout << "[ROSAirSim] Using transform tree structure for ROS "
                "Cartographer."
             << endl;
    }
    cout << " " << endl;

    // initialize ros
    if (!ros::isInitialized()) {
        int argc = 0;
        // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_shared<ros::NodeHandle>();

    // Setup robot namespace
    ros_namespace_ = vehicle_name_;
    // ros_name_ = sc::get<std::string>("ros_namespace_prefix", params,
    // "robot"); ros_namespace_ = ros_name_ +
    // std::to_string(parent_->id().id());

    // setup image transport node
    it_ = std::make_shared<image_transport::ImageTransport>(*nh_);

    // Create TF: Transformation Broadcaster for LIDAR
    //////////////////////////////////////////////////////////////////////
    laser_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    std::vector<geometry_msgs::TransformStamped> tf_msg_vec_;
    tf_msg_vec_.clear();

    // Create World Transform to Robot
    sc::StatePtr &state = parent_->state_truth();
    world_trans_.header.frame_id = "world";
    if (ros_cartographer_) {
        world_trans_.child_frame_id = ros_namespace_;
    } else {
        world_trans_.child_frame_id = ros_namespace_ + "/base_link";
    }
    world_trans_.header.stamp = ros::Time::now();
    world_trans_.transform.translation.x = state->pos().x();
    world_trans_.transform.translation.y = state->pos().y();
    world_trans_.transform.translation.z = state->pos().z();
    world_trans_.transform.rotation.x = state->quat().x();
    world_trans_.transform.rotation.y = state->quat().y();
    world_trans_.transform.rotation.z = state->quat().z();
    world_trans_.transform.rotation.w = state->quat().w();
    tf_msg_vec_.push_back(world_trans_);

    // Create Robot Transform to Base Link -- this allows compatibility for ROS
    // cartographer
    if (ros_cartographer_) {
        vehicle_trans_.header.frame_id = ros_namespace_;
        vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
        vehicle_trans_.header.stamp = ros::Time::now();
        vehicle_trans_.transform.translation.x = 0.0;
        vehicle_trans_.transform.translation.y = 0.0;
        vehicle_trans_.transform.translation.z = 0.0;
        vehicle_trans_.transform.rotation.x = 0.0;
        vehicle_trans_.transform.rotation.y = 0.0;
        vehicle_trans_.transform.rotation.z = 0.0;
        vehicle_trans_.transform.rotation.w = 1.0;
        tf_msg_vec_.push_back(vehicle_trans_);
    }
    laser_broadcaster_->sendTransform(tf_msg_vec_);
    tf_msg_vec_.clear();

    // airsim image callback
    auto airsim_image_cb = [&](auto &msg) {
        image_data_ = msg->data;
        if (image_data_.empty()) {  // return if empty message
            return;
        }

        img_topic_published_mutex_.lock();
        bool img_topic_published = img_topic_published_;
        img_topic_published_mutex_.unlock();

        // If topics are not published create them
        if (!img_topic_published) {
            // for each image in the message
            for (sc::sensor::AirSimImageType a : image_data_) {
                // Create the topic name -- ROS needs lowercase names
                std::string camera_name = boost::algorithm::to_lower_copy(a.camera_config.cam_name);
                std::string image_type_name = boost::algorithm::to_lower_copy(a.camera_config.img_type_name);
                std::string cam_topic_name = ros_namespace_ + "/camera/" + camera_name;
                std::string topic_name_pub = cam_topic_name + "/" + image_type_name;
                std::string info_topic_name_pub = topic_name_pub + "/info";
                topic_name_pub += +"/raw";
                // std::string topic_name = "/" + topic_name_pub;

                if (ros_python_) {
                    img_publishers_.push_back(nh_->advertise<sensor_msgs::Image>(topic_name_pub, 1));
                } else {
                    trans_img_publishers_.push_back(it_->advertise(topic_name_pub, 1));
                }

                // Publish camera info message
                sensor_msgs::CameraInfo info_msg;
                info_msg.header.stamp = ros::Time::now();
                info_msg.header.frame_id = info_topic_name_pub + "_link";
                info_msg.height = a.camera_config.height;
                info_msg.width = a.camera_config.width;
                float cx = a.camera_config.width / 2.0;
                float cy = a.camera_config.height / 2.0;
                double f = (M_PI / 180.0) * (a.camera_config.fov / 2.0);
                float fx = static_cast<float>(tan(f));
                float fy = fx;
                info_msg.distortion_model = "plumb_bob";
                info_msg.K = {fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f};
                info_msg.R = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
                info_msg.P = {fx, 0.f, cx, 0.f, 0.f, fy, cy, 0.f, 0.f, 0.f, 1.f, 0.f};
                cam_info_publishers_.push_back(nh_->advertise<sensor_msgs::CameraInfo>(info_topic_name_pub, 1));
                // publish using last publisher created
                cam_info_publishers_.back().publish(info_msg);

                // Publish current message so we don't miss any images
                sensor_msgs::ImagePtr img_msg;
                std_msgs::Header header;          // empty header
                header.stamp = ros::Time::now();  // time
                header.frame_id = topic_name_pub + "_link";
                // Depth Images (Depth Perspective and Depth Planner) come in as
                // 1 channel float arrays
                if (a.camera_config.pixels_as_float) {
                    img_msg = cv_bridge::CvImage(header, "", a.img).toImageMsg();
                } else {
                    img_msg = cv_bridge::CvImage(header, "rgb8", a.img).toImageMsg();
                }
                // Publish to the last publisher added to img_publishers_
                if (ros_python_) {
                    img_publishers_.back().publish(img_msg);
                } else {
                    trans_img_publishers_.back().publish(img_msg);
                }

                //// publish a transform for each unique camera_name
                // Note down camera names since each camera will need its own
                // image transform old_cam_name=true if camera_name already
                // exists
                bool old_cam_name = std::any_of(camera_names_.begin(), camera_names_.end(),
                                                [camera_name](std::string str) { return str == camera_name; });
                // If false, this is a new camera name, save and publish
                // transform
                if (!old_cam_name) {
                    // cout << "New camera found: " << camera_name << endl;
                    camera_names_.push_back(camera_name);
                    std::vector<geometry_msgs::TransformStamped> tf_msg_vec_;
                    tf_msg_vec_.clear();

                    // Get Vehicle Pose from NED to ENU
                    Eigen::Isometry3f tf_world_vehicle_ENU =
                        get_vehicle_world_pose_from_NED_to_ENU(a.vehicle_pose_world_NED);
                    // Get pose of lidar in ENU, vehicle frame using Eigen
                    Eigen::Isometry3f tf_vehicle_camera_ENU =
                        get_sensor_pose_from_worldNED_to_vehicleENU(a.vehicle_pose_world_NED, a.camera_pose_world_NED);

                    //////////////////////////////////////////////////////////////////////
                    // Update and Publish Transforms
                    //////////////////////////////////////////////////////////////////////
                    world_trans_.header.stamp = ros::Time::now();
                    world_trans_.header.frame_id = "world";
                    if (ros_cartographer_) {
                        world_trans_.child_frame_id = ros_namespace_;
                    } else {
                        world_trans_.child_frame_id = ros_namespace_ + "/base_link";
                    }
                    world_trans_.transform.translation.x = tf_world_vehicle_ENU.translation().x();
                    world_trans_.transform.translation.y = tf_world_vehicle_ENU.translation().y();
                    world_trans_.transform.translation.z = tf_world_vehicle_ENU.translation().z();
                    Eigen::Quaternionf tf_vehicle_pose_rotation(tf_world_vehicle_ENU.rotation());
                    world_trans_.transform.rotation.x = tf_vehicle_pose_rotation.x();
                    world_trans_.transform.rotation.y = tf_vehicle_pose_rotation.y();
                    world_trans_.transform.rotation.z = tf_vehicle_pose_rotation.z();
                    world_trans_.transform.rotation.w = tf_vehicle_pose_rotation.w();
                    world_trans_.header.stamp = ros::Time::now();
                    tf_msg_vec_.push_back(world_trans_);

                    // Send Robot Transform to Base Link
                    if (ros_cartographer_) {
                        vehicle_trans_.header.frame_id = ros_namespace_;
                        vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
                        vehicle_trans_.header.stamp = ros::Time::now();
                        vehicle_trans_.transform.translation.x = 0.0;
                        vehicle_trans_.transform.translation.y = 0.0;
                        vehicle_trans_.transform.translation.z = 0.0;
                        vehicle_trans_.transform.rotation.x = 0.0;
                        vehicle_trans_.transform.rotation.y = 0.0;
                        vehicle_trans_.transform.rotation.z = 0.0;
                        vehicle_trans_.transform.rotation.w = 1.0;
                        tf_msg_vec_.push_back(vehicle_trans_);
                    }

                    // cout << "publishing transform for: " << camera_name <<
                    // endl; Publish Image transform to broadcaster
                    geometry_msgs::TransformStamped image_trans_;
                    image_trans_.header.frame_id = ros_namespace_ + "/base_link";
                    image_trans_.child_frame_id = cam_topic_name + "_link";
                    image_trans_.header.stamp = ros::Time::now();

                    // Position
                    image_trans_.transform.translation.x = tf_vehicle_camera_ENU.translation().x();
                    image_trans_.transform.translation.y = tf_vehicle_camera_ENU.translation().y();
                    image_trans_.transform.translation.z = tf_vehicle_camera_ENU.translation().z();
                    // Orientation
                    Eigen::Quaternionf tf_vehicle_camera_ENU_rotation(tf_vehicle_camera_ENU.rotation());
                    image_trans_.transform.rotation.w = tf_vehicle_camera_ENU_rotation.w();
                    image_trans_.transform.rotation.x = tf_vehicle_camera_ENU_rotation.x();
                    image_trans_.transform.rotation.y = tf_vehicle_camera_ENU_rotation.y();
                    image_trans_.transform.rotation.z = tf_vehicle_camera_ENU_rotation.z();
                    tf_msg_vec_.push_back(image_trans_);

                    laser_broadcaster_->sendTransform(tf_msg_vec_);
                    tf_msg_vec_.clear();
                }  // end if new camera name to publish pose for
            }  // end for loop a : images in msg
            img_topic_published_mutex_.lock();
            img_topic_published_ = true;
            img_topic_published_mutex_.unlock();

            return;  // return from callback because the first set of images are
                     // already published
        }  // end if img_topic_published=false
    };

    //// airsim lidar callback
    auto airsim_lidar_cb = [&](auto &msg) {
        lidar_data_ = msg->data;
        if (lidar_data_.empty()) {  // return if empty message
            return;
        }

        lidar_topic_published_mutex_.lock();
        bool lidar_topic_published = lidar_topic_published_;
        lidar_topic_published_mutex_.unlock();

        // If topics are not published create them
        if (!lidar_topic_published) {
            // for each image in the message
            for (sc::sensor::AirSimLidarType l : lidar_data_) {
                // Create the topic name
                std::string lidar_name = boost::algorithm::to_lower_copy(l.lidar_name);
                std::string topic_name_pub = ros_namespace_ + "/" + lidar_name;
                // std::string topic_name = "/" + topic_name_pub;
                lidar_publishers_.push_back(nh_->advertise<sensor_msgs::PointCloud2>(topic_name_pub, 1));
                sensor_msgs::PointCloud2 lidar_msg;
                lidar_msg.header.stamp = ros::Time::now();
                lidar_msg.header.frame_id = topic_name_pub + "_link";

                if (l.lidar_data.point_cloud.size() > 3) {
                    lidar_msg.height = 1;
                    lidar_msg.width = l.lidar_data.point_cloud.size() / 3;

                    lidar_msg.fields.resize(3);
                    lidar_msg.fields[0].name = "x";
                    lidar_msg.fields[1].name = "y";
                    lidar_msg.fields[2].name = "z";
                    int offset = 0;

                    for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
                        lidar_msg.fields[d].offset = offset;
                        lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
                        lidar_msg.fields[d].count = 1;
                    }

                    lidar_msg.is_bigendian = false;
                    lidar_msg.point_step = offset;  // 4 * num fields
                    lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

                    lidar_msg.is_dense = true;
                    std::vector<float> data_std = l.lidar_data.point_cloud;

                    const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&data_std[0]);
                    vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
                    lidar_msg.data = std::move(lidar_msg_data);
                }

                // Publish to the last publisher added to lidar_publishers_
                lidar_publishers_.back().publish(lidar_msg);

                //// publish a transform for each lidar
                // Note down imu names since each imu will need its own
                // transform
                lidar_names_.push_back(l.lidar_name);
                std::vector<geometry_msgs::TransformStamped> tf_msg_vec_;
                tf_msg_vec_.clear();

                // Get Vehicle Pose from NED to ENU
                Eigen::Isometry3f tf_world_vehicle_ENU =
                    get_vehicle_world_pose_from_NED_to_ENU(l.vehicle_pose_world_NED);
                // Get pose of lidar in ENU, vehicle frame using Eigen
                Eigen::Isometry3f tf_vehicle_lidar_ENU =
                    get_sensor_pose_from_worldNED_to_vehicleENU(l.vehicle_pose_world_NED, l.lidar_pose_world_NED);

                //////////////////////////////////////////////////////////////////////
                // Update and Publish Transforms
                //////////////////////////////////////////////////////////////////////
                world_trans_.header.stamp = ros::Time::now();
                world_trans_.header.frame_id = "world";
                if (ros_cartographer_) {
                    world_trans_.child_frame_id = ros_namespace_;
                } else {
                    world_trans_.child_frame_id = ros_namespace_ + "/base_link";
                }
                world_trans_.transform.translation.x = tf_world_vehicle_ENU.translation().x();
                world_trans_.transform.translation.y = tf_world_vehicle_ENU.translation().y();
                world_trans_.transform.translation.z = tf_world_vehicle_ENU.translation().z();
                Eigen::Quaternionf tf_vehicle_pose_rotation(tf_world_vehicle_ENU.rotation());
                world_trans_.transform.rotation.x = tf_vehicle_pose_rotation.x();
                world_trans_.transform.rotation.y = tf_vehicle_pose_rotation.y();
                world_trans_.transform.rotation.z = tf_vehicle_pose_rotation.z();
                world_trans_.transform.rotation.w = tf_vehicle_pose_rotation.w();
                world_trans_.header.stamp = ros::Time::now();
                tf_msg_vec_.push_back(world_trans_);

                // Send Robot Transform to Base Link
                if (ros_cartographer_) {
                    vehicle_trans_.header.frame_id = ros_namespace_;
                    vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
                    vehicle_trans_.header.stamp = ros::Time::now();
                    vehicle_trans_.transform.translation.x = 0.0;
                    vehicle_trans_.transform.translation.y = 0.0;
                    vehicle_trans_.transform.translation.z = 0.0;
                    vehicle_trans_.transform.rotation.x = 0.0;
                    vehicle_trans_.transform.rotation.y = 0.0;
                    vehicle_trans_.transform.rotation.z = 0.0;
                    vehicle_trans_.transform.rotation.w = 1.0;
                    tf_msg_vec_.push_back(vehicle_trans_);
                }

                // Publish LIDAR transform to broadcaster
                geometry_msgs::TransformStamped lidar_trans_;
                lidar_trans_.header.frame_id = ros_namespace_ + "/base_link";
                lidar_trans_.child_frame_id = topic_name_pub + "_link";
                lidar_trans_.header.stamp = ros::Time::now();

                // Position
                lidar_trans_.transform.translation.x = tf_vehicle_lidar_ENU.translation().x();
                lidar_trans_.transform.translation.y = tf_vehicle_lidar_ENU.translation().y();
                lidar_trans_.transform.translation.z = tf_vehicle_lidar_ENU.translation().z();
                // Orientation
                Eigen::Quaternionf tf_vehicle_lidar_ENU_rotation(tf_vehicle_lidar_ENU.rotation());
                lidar_trans_.transform.rotation.w = tf_vehicle_lidar_ENU_rotation.w();
                lidar_trans_.transform.rotation.x = tf_vehicle_lidar_ENU_rotation.x();
                lidar_trans_.transform.rotation.y = tf_vehicle_lidar_ENU_rotation.y();
                lidar_trans_.transform.rotation.z = tf_vehicle_lidar_ENU_rotation.z();
                tf_msg_vec_.push_back(lidar_trans_);

                laser_broadcaster_->sendTransform(tf_msg_vec_);
                tf_msg_vec_.clear();
            }  // end for loop i : imus in msg
            lidar_topic_published_mutex_.lock();
            lidar_topic_published_ = true;
            lidar_topic_published_mutex_.unlock();

            return;  // return from callback because the first set of images are
                     // already published
        }  // end if img_topic_published=false
    };

    //// airsim imu callback
    auto airsim_imu_cb = [&](auto &msg) {
        imu_data_ = msg->data;
        if (imu_data_.empty()) {  // return if empty message
            return;
        }

        imu_topic_published_mutex_.lock();
        bool imu_topic_published = imu_topic_published_;
        imu_topic_published_mutex_.unlock();

        // If topics are not published create them
        if (!imu_topic_published) {
            // for each image in the message
            for (sc::sensor::AirSimImuType i : imu_data_) {
                // Create the topic name
                std::string imu_name = boost::algorithm::to_lower_copy(i.imu_name);
                std::string topic_name_pub = ros_namespace_ + "/" + imu_name;
                // std::string topic_name = "/" + topic_name_pub;
                imu_publishers_.push_back(nh_->advertise<sensor_msgs::Imu>(topic_name_pub, 1));

                // Publish current message so we don't miss any imu data
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = topic_name_pub + "_link";
                imu_msg.orientation.x = i.imu_data.orientation.x();
                imu_msg.orientation.y = i.imu_data.orientation.y();
                imu_msg.orientation.z = i.imu_data.orientation.z();
                imu_msg.orientation.w = i.imu_data.orientation.w();
                imu_msg.angular_velocity.x = i.imu_data.angular_velocity.x();
                imu_msg.angular_velocity.y = i.imu_data.angular_velocity.y();
                imu_msg.angular_velocity.z = i.imu_data.angular_velocity.z();
                imu_msg.linear_acceleration.x = i.imu_data.linear_acceleration.x();
                imu_msg.linear_acceleration.y = i.imu_data.linear_acceleration.y();
                imu_msg.linear_acceleration.z = i.imu_data.linear_acceleration.z();
                // Publish to the last publisher added to imu_publishers_
                imu_publishers_.back().publish(imu_msg);

                //// publish a transform for each unique imu_name
                // Note down imu names since each imu will need its own
                // transform
                imu_names_.push_back(i.imu_name);
                std::vector<geometry_msgs::TransformStamped> tf_msg_vec_;
                tf_msg_vec_.clear();

                // Get Vehicle Pose from NED to ENU
                Eigen::Isometry3f tf_world_vehicle_ENU =
                    get_vehicle_world_pose_from_NED_to_ENU(i.vehicle_pose_world_NED);
                // Get pose of lidar in ENU, vehicle frame using Eigen
                Eigen::Isometry3f tf_vehicle_imu_ENU =
                    get_sensor_pose_from_worldNED_to_vehicleENU(i.vehicle_pose_world_NED, i.imu_pose_world_NED);

                //////////////////////////////////////////////////////////////////////
                // Update and Publish Transforms
                //////////////////////////////////////////////////////////////////////
                world_trans_.header.stamp = ros::Time::now();
                world_trans_.header.frame_id = "world";
                if (ros_cartographer_) {
                    world_trans_.child_frame_id = ros_namespace_;
                } else {
                    world_trans_.child_frame_id = ros_namespace_ + "/base_link";
                }
                world_trans_.transform.translation.x = tf_world_vehicle_ENU.translation().x();
                world_trans_.transform.translation.y = tf_world_vehicle_ENU.translation().y();
                world_trans_.transform.translation.z = tf_world_vehicle_ENU.translation().z();
                Eigen::Quaternionf tf_vehicle_pose_rotation(tf_world_vehicle_ENU.rotation());
                world_trans_.transform.rotation.x = tf_vehicle_pose_rotation.x();
                world_trans_.transform.rotation.y = tf_vehicle_pose_rotation.y();
                world_trans_.transform.rotation.z = tf_vehicle_pose_rotation.z();
                world_trans_.transform.rotation.w = tf_vehicle_pose_rotation.w();
                world_trans_.header.stamp = ros::Time::now();
                tf_msg_vec_.push_back(world_trans_);

                if (ros_cartographer_) {
                    // Send Robot Transform to Base Link
                    vehicle_trans_.header.frame_id = ros_namespace_;
                    vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
                    vehicle_trans_.header.stamp = ros::Time::now();
                    vehicle_trans_.transform.translation.x = 0.0;
                    vehicle_trans_.transform.translation.y = 0.0;
                    vehicle_trans_.transform.translation.z = 0.0;
                    vehicle_trans_.transform.rotation.x = 0.0;
                    vehicle_trans_.transform.rotation.y = 0.0;
                    vehicle_trans_.transform.rotation.z = 0.0;
                    vehicle_trans_.transform.rotation.w = 1.0;
                    tf_msg_vec_.push_back(vehicle_trans_);
                }

                // Publish Imu transform to broadcaster
                geometry_msgs::TransformStamped imu_trans_;
                imu_trans_.header.frame_id = ros_namespace_ + "/base_link";
                imu_trans_.child_frame_id = topic_name_pub + "_link";
                imu_trans_.header.stamp = ros::Time::now();

                // Position
                imu_trans_.transform.translation.x = tf_vehicle_imu_ENU.translation().x();
                imu_trans_.transform.translation.y = tf_vehicle_imu_ENU.translation().y();
                imu_trans_.transform.translation.z = tf_vehicle_imu_ENU.translation().z();
                // Orientation
                Eigen::Quaternionf tf_vehicle_imu_ENU_rotation(tf_vehicle_imu_ENU.rotation());
                imu_trans_.transform.rotation.w = tf_vehicle_imu_ENU_rotation.w();
                imu_trans_.transform.rotation.x = tf_vehicle_imu_ENU_rotation.x();
                imu_trans_.transform.rotation.y = tf_vehicle_imu_ENU_rotation.y();
                imu_trans_.transform.rotation.z = tf_vehicle_imu_ENU_rotation.z();
                tf_msg_vec_.push_back(imu_trans_);

                laser_broadcaster_->sendTransform(tf_msg_vec_);
                tf_msg_vec_.clear();
            }  // end for loop i : imus in msg
            imu_topic_published_mutex_.lock();
            imu_topic_published_ = true;
            imu_topic_published_mutex_.unlock();

            return;  // return from callback because the first set of images are
                     // already published
        }  // end if img_topic_published=false
    };

    if (pub_imu_data_) {
        subscribe<std::vector<sensor::AirSimImuType>>("LocalNetwork", "AirSimImu", airsim_imu_cb);
    }
    if (pub_lidar_data_) {
        subscribe<std::vector<sensor::AirSimLidarType>>("LocalNetwork", "AirSimLidar", airsim_lidar_cb);
    }
    if (pub_image_data_) {
        subscribe<std::vector<sensor::AirSimImageType>>("LocalNetwork", "AirSimImages", airsim_image_cb);
    }
}

bool ROSAirSim::step_autonomy(double t, double dt) {
    ros::spinOnce();  // check for new ROS messages
    std::vector<geometry_msgs::TransformStamped> tf_msg_vec_;
    tf_msg_vec_.clear();

    //// Update Imus
    //////////////////////////////////////////////////////////////////////
    imu_topic_published_mutex_.lock();
    bool imu_topic_published = imu_topic_published_;
    imu_topic_published_mutex_.unlock();

    if (pub_imu_data_ && imu_topic_published) {
        // All image topics should be published
        for (sc::sensor::AirSimImuType i : imu_data_) {
            // create string to match using topic name
            std::string imu_name = boost::algorithm::to_lower_copy(i.imu_name);
            std::string topic_name_pub = ros_namespace_ + "/" + imu_name;
            std::string topic_name_match = "/" + topic_name_pub;
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = topic_name_pub + "_link";

            // cout << "imu_publishers_ length: " << imu_publishers_.size() <<
            // endl;
            for (auto pub : imu_publishers_) {
                // If the topic publisher exists, publish to topic
                if (pub.getTopic() == topic_name_match) {
                    imu_msg.orientation.x = i.imu_data.orientation.x();
                    imu_msg.orientation.y = i.imu_data.orientation.y();
                    imu_msg.orientation.z = i.imu_data.orientation.z();
                    imu_msg.orientation.w = i.imu_data.orientation.w();
                    imu_msg.angular_velocity.x = i.imu_data.angular_velocity.x();
                    imu_msg.angular_velocity.y = i.imu_data.angular_velocity.y();
                    imu_msg.angular_velocity.z = i.imu_data.angular_velocity.z();
                    imu_msg.linear_acceleration.x = i.imu_data.linear_acceleration.x();
                    imu_msg.linear_acceleration.y = i.imu_data.linear_acceleration.y();
                    imu_msg.linear_acceleration.z = i.imu_data.linear_acceleration.z();
                    pub.publish(imu_msg);
                    break;  // break publish for loop after image is published
                }  // end if topic name matches
            }  // end publishers for loop

            //// for each imu publish a transform
            // Get Vehicle Pose from NED to ENU
            Eigen::Isometry3f tf_world_vehicle_ENU = get_vehicle_world_pose_from_NED_to_ENU(i.vehicle_pose_world_NED);
            // Get pose of imu in ENU, vehicle frame using Eigen
            Eigen::Isometry3f tf_vehicle_imu_ENU =
                get_sensor_pose_from_worldNED_to_vehicleENU(i.vehicle_pose_world_NED, i.imu_pose_world_NED);

            // cout << "If not publishing lidar or image data, publish world
            // frame from imu" << endl;
            //////////////////////////////////////////////////////////////////////
            // Update and Publish Transform
            //////////////////////////////////////////////////////////////////////
            // Publish Vehicle Pose using IMU sensor because it has the least
            // receiving/ processing time
            world_trans_.header.stamp = ros::Time::now();
            world_trans_.header.frame_id = "world";
            if (ros_cartographer_) {
                world_trans_.child_frame_id = ros_namespace_;
            } else {
                world_trans_.child_frame_id = ros_namespace_ + "/base_link";
            }
            world_trans_.transform.translation.x = tf_world_vehicle_ENU.translation().x();
            world_trans_.transform.translation.y = tf_world_vehicle_ENU.translation().y();
            world_trans_.transform.translation.z = tf_world_vehicle_ENU.translation().z();
            Eigen::Quaternionf tf_vehicle_pose_rotation(tf_world_vehicle_ENU.rotation());
            world_trans_.transform.rotation.x = tf_vehicle_pose_rotation.x();
            world_trans_.transform.rotation.y = tf_vehicle_pose_rotation.y();
            world_trans_.transform.rotation.z = tf_vehicle_pose_rotation.z();
            world_trans_.transform.rotation.w = tf_vehicle_pose_rotation.w();
            world_trans_.header.stamp = ros::Time::now();
            tf_msg_vec_.push_back(world_trans_);

            if (ros_cartographer_) {
                // Send Robot Transform to Base Link
                vehicle_trans_.header.frame_id = ros_namespace_;
                vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
                vehicle_trans_.header.stamp = ros::Time::now();
                vehicle_trans_.transform.translation.x = 0.0;
                vehicle_trans_.transform.translation.y = 0.0;
                vehicle_trans_.transform.translation.z = 0.0;
                vehicle_trans_.transform.rotation.x = 0.0;
                vehicle_trans_.transform.rotation.y = 0.0;
                vehicle_trans_.transform.rotation.z = 0.0;
                vehicle_trans_.transform.rotation.w = 1.0;
                tf_msg_vec_.push_back(vehicle_trans_);
            }

            // Publish Imu transform to broadcaster
            geometry_msgs::TransformStamped imu_trans_;
            imu_trans_.header.frame_id = ros_namespace_ + "/base_link";
            imu_trans_.child_frame_id = topic_name_pub + "_link";
            imu_trans_.header.stamp = ros::Time::now();
            // Position
            imu_trans_.transform.translation.x = tf_vehicle_imu_ENU.translation().x();
            imu_trans_.transform.translation.y = tf_vehicle_imu_ENU.translation().y();
            imu_trans_.transform.translation.z = tf_vehicle_imu_ENU.translation().z();
            // Orientation
            Eigen::Quaternionf tf_vehicle_imu_ENU_rotation(tf_vehicle_imu_ENU.rotation());
            imu_trans_.transform.rotation.w = tf_vehicle_imu_ENU_rotation.w();
            imu_trans_.transform.rotation.x = tf_vehicle_imu_ENU_rotation.x();
            imu_trans_.transform.rotation.y = tf_vehicle_imu_ENU_rotation.y();
            imu_trans_.transform.rotation.z = tf_vehicle_imu_ENU_rotation.z();
            tf_msg_vec_.push_back(imu_trans_);
        }  // end imus in message for loop
    }

    //// Update LIDAR PointCloud2 ROS message
    //////////////////////////////////////////////////////////////////////
    lidar_topic_published_mutex_.lock();
    bool lidar_topic_published = lidar_topic_published_;
    lidar_topic_published_mutex_.unlock();

    if (pub_lidar_data_ && lidar_topic_published) {
        // All image topics should be published
        for (sc::sensor::AirSimLidarType l : lidar_data_) {
            // Create the topic name
            std::string lidar_name = boost::algorithm::to_lower_copy(l.lidar_name);
            std::string topic_name_pub = ros_namespace_ + "/" + lidar_name;
            std::string topic_name_match = "/" + topic_name_pub;
            sensor_msgs::PointCloud2 lidar_msg;
            lidar_msg.header.stamp = ros::Time::now();
            lidar_msg.header.frame_id = topic_name_pub + "_link";

            // cout << "imu_publishers_ length: " << imu_publishers_.size() <<
            // endl;
            for (auto pub : lidar_publishers_) {
                // If the topic publisher exists, publish to topic
                if (pub.getTopic() == topic_name_match) {
                    if (l.lidar_data.point_cloud.size() > 3) {
                        lidar_msg.height = 1;
                        lidar_msg.width = l.lidar_data.point_cloud.size() / 3;

                        lidar_msg.fields.resize(3);
                        lidar_msg.fields[0].name = "x";
                        lidar_msg.fields[1].name = "y";
                        lidar_msg.fields[2].name = "z";
                        int offset = 0;

                        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
                            lidar_msg.fields[d].offset = offset;
                            lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
                            lidar_msg.fields[d].count = 1;
                        }

                        lidar_msg.is_bigendian = false;
                        lidar_msg.point_step = offset;  // 4 * num fields
                        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

                        lidar_msg.is_dense = true;
                        std::vector<float> data_std = l.lidar_data.point_cloud;

                        const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&data_std[0]);
                        vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
                        lidar_msg.data = std::move(lidar_msg_data);
                    }
                    pub.publish(lidar_msg);
                    break;  // break publish for loop after lidar data is
                            // published
                }  // end if topic name matches
            }  // end publishers for loop

            //// publish a transform for each lidar
            // Get Vehicle Pose from NED to ENU
            Eigen::Isometry3f tf_world_vehicle_ENU = get_vehicle_world_pose_from_NED_to_ENU(l.vehicle_pose_world_NED);
            // Get pose of lidar in ENU, vehicle frame using Eigen
            Eigen::Isometry3f tf_vehicle_lidar_ENU =
                get_sensor_pose_from_worldNED_to_vehicleENU(l.vehicle_pose_world_NED, l.lidar_pose_world_NED);

            // If not publishing imu, publish world frame from lidar -- 2nd
            // least receiving/ processing time.
            if (!pub_imu_data_) {
                // cout << "If not publishing imu data, publish world frame from
                // lidar"
                // << endl;
                //////////////////////////////////////////////////////////////////////
                // Update and Publish Transforms
                //////////////////////////////////////////////////////////////////////
                world_trans_.header.stamp = ros::Time::now();
                world_trans_.header.frame_id = "world";
                if (ros_cartographer_) {
                    world_trans_.child_frame_id = ros_namespace_;
                } else {
                    world_trans_.child_frame_id = ros_namespace_ + "/base_link";
                }
                world_trans_.transform.translation.x = tf_world_vehicle_ENU.translation().x();
                world_trans_.transform.translation.y = tf_world_vehicle_ENU.translation().y();
                world_trans_.transform.translation.z = tf_world_vehicle_ENU.translation().z();
                Eigen::Quaternionf tf_vehicle_pose_rotation(tf_world_vehicle_ENU.rotation());
                world_trans_.transform.rotation.x = tf_vehicle_pose_rotation.x();
                world_trans_.transform.rotation.y = tf_vehicle_pose_rotation.y();
                world_trans_.transform.rotation.z = tf_vehicle_pose_rotation.z();
                world_trans_.transform.rotation.w = tf_vehicle_pose_rotation.w();
                tf_msg_vec_.push_back(world_trans_);

                if (ros_cartographer_) {
                    // Send Robot Transform to Base Link
                    vehicle_trans_.header.frame_id = ros_namespace_;
                    vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
                    vehicle_trans_.header.stamp = ros::Time::now();
                    vehicle_trans_.transform.translation.x = 0.0;
                    vehicle_trans_.transform.translation.y = 0.0;
                    vehicle_trans_.transform.translation.z = 0.0;
                    vehicle_trans_.transform.rotation.x = 0.0;
                    vehicle_trans_.transform.rotation.y = 0.0;
                    vehicle_trans_.transform.rotation.z = 0.0;
                    vehicle_trans_.transform.rotation.w = 1.0;
                    tf_msg_vec_.push_back(vehicle_trans_);
                }
            }
            // Publish Lidar transform to broadcaster
            geometry_msgs::TransformStamped lidar_trans_;
            lidar_trans_.header.frame_id = ros_namespace_ + "/base_link";
            lidar_trans_.child_frame_id = topic_name_pub + "_link";
            lidar_trans_.header.stamp = ros::Time::now();

            // Position
            lidar_trans_.transform.translation.x = tf_vehicle_lidar_ENU.translation().x();
            lidar_trans_.transform.translation.y = tf_vehicle_lidar_ENU.translation().y();
            lidar_trans_.transform.translation.z = tf_vehicle_lidar_ENU.translation().z();
            // Orientation
            Eigen::Quaternionf tf_vehicle_lidar_ENU_rotation(tf_vehicle_lidar_ENU.rotation());
            lidar_trans_.transform.rotation.w = tf_vehicle_lidar_ENU_rotation.w();
            lidar_trans_.transform.rotation.x = tf_vehicle_lidar_ENU_rotation.x();
            lidar_trans_.transform.rotation.y = tf_vehicle_lidar_ENU_rotation.y();
            lidar_trans_.transform.rotation.z = tf_vehicle_lidar_ENU_rotation.z();
            tf_msg_vec_.push_back(lidar_trans_);
        }  // end lidars in message for loop
    }

    //// Update Images
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
            std::string topic_name_pub = ros_namespace_ + "/camera/" + camera_name + "/" + image_type_name;
            // std::string info_topic_name_pub = topic_name_pub + "/info";
            topic_name_pub += +"/raw";
            std::string topic_name_match = "/" + topic_name_pub;

            std_msgs::Header header;          // empty header
            header.stamp = ros::Time::now();  // time
            header.frame_id = topic_name_pub + "_link";
            sensor_msgs::ImagePtr img_msg;

            // cout << "img_publishers_ length: " << img_publishers_.size() <<
            // endl;
            if (ros_python_) {
                for (auto pub : img_publishers_) {
                    // If the topic publisher exists, publish to topic
                    if (pub.getTopic() == topic_name_match) {
                        // Depth Images (Depth Perspective and Depth Planner)
                        // come in as 1 channel float arrays
                        if (a.camera_config.pixels_as_float) {
                            img_msg = cv_bridge::CvImage(header, "", a.img).toImageMsg();
                        } else {
                            img_msg = cv_bridge::CvImage(header, "rgb8", a.img).toImageMsg();
                        }
                        pub.publish(img_msg);
                        break;  // break publish for loop after image is
                                // published
                    }  // end if topic name matches
                }  // end publishers for loop
            } else {
                for (auto pub : trans_img_publishers_) {
                    // If the topic publisher exists, publish to topic
                    if (pub.getTopic() == topic_name_match) {
                        // Depth Images (Depth Perspective and Depth Planner)
                        // come in as 1 channel float arrays
                        if (a.camera_config.pixels_as_float) {
                            img_msg = cv_bridge::CvImage(header, "", a.img).toImageMsg();
                        } else {
                            img_msg = cv_bridge::CvImage(header, "rgb8", a.img).toImageMsg();
                        }
                        pub.publish(img_msg);
                        break;  // break publish for loop after image is
                                // published
                    }  // end if topic name matches
                }  // end publishers for loop
            }

            // draw image
            if (show_camera_images_) {
                std::string window_name =
                    a.vehicle_name + "_" + a.camera_config.cam_name + "_" + a.camera_config.img_type_name;
                if (a.camera_config.pixels_as_float) {
                    cv::Mat tempImage;
                    a.img.convertTo(tempImage, CV_32FC1, 1.f / 255);
                    // cv::normalize(a.img, tempImage, 0, 1, cv::NORM_MINMAso
                    // use if to determine.X); cout << tempImage << endl;
                    cv::imshow(window_name, tempImage);
                } else {
                    // other image types are int 0-255.
                    if (a.img.channels() == 4) {
                        cout << "image channels: " << a.img.channels() << endl;
                        cout << "Warning: Old AirSim Linux Asset Environments "
                                "have 4 "
                                "channels. Color images will not display "
                                "correctly."
                             << endl;
                        cout << "Warning: Use Asset Environment versions "
                                "Linux-v1.3.1+."
                             << endl;
                        cv::Mat tempImage;
                        cv::cvtColor(a.img, tempImage, CV_RGBA2RGB);
                        cv::imshow(window_name, tempImage);
                    } else {
                        cv::imshow(window_name, a.img);
                    }
                }
                cv::waitKey(1);
            }  // end draw image
        }  // end images in message for loop

        // for each camera_name publish a transform
        // cout << "camera_names length: " << camera_names_.size() << endl;
        for (std::string cam_name : camera_names_) {
            for (sc::sensor::AirSimImageType a : image_data_) {
                if (a.camera_config.cam_name == cam_name) {
                    // cout << "publishing transform for: " << cam_name << endl;
                    // create string to match using topic name
                    std::string camera_name = boost::algorithm::to_lower_copy(a.camera_config.cam_name);
                    std::string cam_topic_name = ros_namespace_ + "/camera/" + camera_name;

                    // Get Vehicle Pose from NED to ENU
                    Eigen::Isometry3f tf_world_vehicle_ENU =
                        get_vehicle_world_pose_from_NED_to_ENU(a.vehicle_pose_world_NED);
                    // Get pose of camera in ENU, vehicle frame using Eigen
                    Eigen::Isometry3f tf_vehicle_camera_ENU =
                        get_sensor_pose_from_worldNED_to_vehicleENU(a.vehicle_pose_world_NED, a.camera_pose_world_NED);
                    // cout << "publishing transform for: " << cam_name << endl;

                    // If not publishing lidar or imu data, publish world frame
                    // from images
                    if (!pub_imu_data_ && !pub_lidar_data_) {
                        //////////////////////////////////////////////////////////////////////
                        // Update and Publish Transforms
                        //////////////////////////////////////////////////////////////////////
                        world_trans_.header.stamp = ros::Time::now();
                        world_trans_.header.frame_id = "world";
                        if (ros_cartographer_) {
                            world_trans_.child_frame_id = ros_namespace_;
                        } else {
                            world_trans_.child_frame_id = ros_namespace_ + "/base_link";
                        }
                        world_trans_.transform.translation.x = tf_world_vehicle_ENU.translation().x();
                        world_trans_.transform.translation.y = tf_world_vehicle_ENU.translation().y();
                        world_trans_.transform.translation.z = tf_world_vehicle_ENU.translation().z();
                        Eigen::Quaternionf tf_vehicle_pose_rotation(tf_world_vehicle_ENU.rotation());
                        world_trans_.transform.rotation.x = tf_vehicle_pose_rotation.x();
                        world_trans_.transform.rotation.y = tf_vehicle_pose_rotation.y();
                        world_trans_.transform.rotation.z = tf_vehicle_pose_rotation.z();
                        world_trans_.transform.rotation.w = tf_vehicle_pose_rotation.w();
                        tf_msg_vec_.push_back(world_trans_);

                        if (ros_cartographer_) {
                            // Send Robot Transform to Base Link
                            vehicle_trans_.header.frame_id = ros_namespace_;
                            vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
                            vehicle_trans_.header.stamp = ros::Time::now();
                            vehicle_trans_.transform.translation.x = 0.0;
                            vehicle_trans_.transform.translation.y = 0.0;
                            vehicle_trans_.transform.translation.z = 0.0;
                            vehicle_trans_.transform.rotation.x = 0.0;
                            vehicle_trans_.transform.rotation.y = 0.0;
                            vehicle_trans_.transform.rotation.z = 0.0;
                            vehicle_trans_.transform.rotation.w = 1.0;
                            tf_msg_vec_.push_back(vehicle_trans_);
                        }
                    }

                    // Publish Image transform to broadcaster
                    geometry_msgs::TransformStamped image_trans_;
                    image_trans_.header.frame_id = ros_namespace_ + "/base_link";
                    image_trans_.child_frame_id = cam_topic_name + "_link";
                    image_trans_.header.stamp = ros::Time::now();

                    // Pose in image_data.pose is in NED, but scrimmage is in
                    // ENU so use the conversion done in AirSimSensor Position
                    image_trans_.transform.translation.x = tf_vehicle_camera_ENU.translation().x();
                    image_trans_.transform.translation.y = tf_vehicle_camera_ENU.translation().y();
                    image_trans_.transform.translation.z = tf_vehicle_camera_ENU.translation().z();
                    // Orientation
                    Eigen::Quaternionf tf_vehicle_camera_ENU_rotation(tf_vehicle_camera_ENU.rotation());
                    image_trans_.transform.rotation.w = tf_vehicle_camera_ENU_rotation.w();
                    image_trans_.transform.rotation.x = tf_vehicle_camera_ENU_rotation.x();
                    image_trans_.transform.rotation.y = tf_vehicle_camera_ENU_rotation.y();
                    image_trans_.transform.rotation.z = tf_vehicle_camera_ENU_rotation.z();
                    tf_msg_vec_.push_back(image_trans_);

                    break;  // break the image in msg for loop if camera_name is
                            // found and move to next camera_name
                }
            }
        }
    }

    // If none of the sensor msgs are being published use parent_->state_truth()
    // vehicle pose
    if (!pub_image_data_ && !pub_lidar_data_ && !pub_imu_data_) {
        sc::StatePtr &state = parent_->state_truth();

        world_trans_.header.stamp = ros::Time::now();
        world_trans_.header.frame_id = "world";
        if (ros_cartographer_) {
            world_trans_.child_frame_id = ros_namespace_;
        } else {
            world_trans_.child_frame_id = ros_namespace_ + "/base_link";
        }
        world_trans_.transform.translation.x = state->pos().x();
        world_trans_.transform.translation.y = state->pos().y();
        world_trans_.transform.translation.z = state->pos().z();
        world_trans_.transform.rotation.x = state->quat().x();
        world_trans_.transform.rotation.y = state->quat().y();
        world_trans_.transform.rotation.z = state->quat().z();
        world_trans_.transform.rotation.w = state->quat().w();
        world_trans_.header.stamp = ros::Time::now();
        tf_msg_vec_.push_back(world_trans_);

        if (ros_cartographer_) {
            // Send Robot Transform to Base Link
            vehicle_trans_.header.frame_id = ros_namespace_;
            vehicle_trans_.child_frame_id = ros_namespace_ + "/base_link";
            vehicle_trans_.header.stamp = ros::Time::now();
            vehicle_trans_.transform.translation.x = 0.0;
            vehicle_trans_.transform.translation.y = 0.0;
            vehicle_trans_.transform.translation.z = 0.0;
            vehicle_trans_.transform.rotation.x = 0.0;
            vehicle_trans_.transform.rotation.y = 0.0;
            vehicle_trans_.transform.rotation.z = 0.0;
            vehicle_trans_.transform.rotation.w = 1.0;
            tf_msg_vec_.push_back(vehicle_trans_);
        }
    }
    laser_broadcaster_->sendTransform(tf_msg_vec_);
    tf_msg_vec_.clear();

    return true;
}
}  // namespace autonomy
}  // namespace scrimmage
