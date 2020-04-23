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
 * @author Natalie Rakoski <natalie.rakoski@gtri.gatech.edu>
 * @date 7 January 2020
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/Angles.h>

#include <iostream>
#include <memory>
#include <limits>
#include <chrono> // NOLINT
#include <thread> // NOLINT
#include <mutex> // NOLINT

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp> // for boost::split
#include <boost/filesystem.hpp>

#include "common/AirSimSettings.hpp"

using std::cout;
using std::endl;

using ang = scrimmage::Angles;

namespace sc = scrimmage;
namespace ma = msr::airlib;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::AirSimSensor, AirSimSensor_plugin)

namespace scrimmage {
namespace sensor {

AirSimSensor::AirSimSensor() : client_connected_(false),
    airsim_ip_("localhost"), airsim_port_(41451), airsim_timeout_s_(60) {

    enu_to_ned_yaw_.set_input_clock_direction(ang::Rotate::CCW);
    enu_to_ned_yaw_.set_input_zero_axis(ang::HeadingZero::Pos_X);
    enu_to_ned_yaw_.set_output_clock_direction(ang::Rotate::CW);
    enu_to_ned_yaw_.set_output_zero_axis(ang::HeadingZero::Pos_Y);
}

void AirSimSensor::parse_camera_configs(std::map<std::string, std::string> &params) {
    // Parse the camera config string.
    // The string is a list of camera configs from AirSimSensor.xml of the form:
    // [CameraName=0 ImageTypeName=Scene ImageTypeNumber=0 Width=256 Height=144]
    std::string camera_config = sc::get<std::string>("camera_config", params, "");
    std::vector<std::string> tokens_1;
    boost::split(tokens_1, camera_config, boost::is_any_of("[]"));
    for (std::string &t_1 : tokens_1) {
        if (t_1.length() > 0) {
            std::vector<std::string> tokens_2;
            boost::split(tokens_2, t_1, boost::is_any_of(" ,"));

            if (tokens_2.size() == 4) {
                try {
                    CameraConfig c;
                    // Get Camera Number Type
                    // Front_Center=0, Front_Right=1, Front_Left=2, Bottom_Center=3, Back_Center=4
                    // c.cam_number = boost::lexical_cast<int>(tokens_2[0]);
                    c.cam_name = tokens_2[0];

                    // Get Image Type Name and Number
                    // c.img_type_number = boost::lexical_cast<int>(tokens_2[2]);
                    // Scene=0, DepthPlanner=1, DepthPerspective=2, DepthVis=3, DisparityNormalized=4,
                    // Segmentation=5, SurfaceNormals=6, Infrared=7
                    if (tokens_2[1] == "Scene") {
                        c.img_type_number = 0;
                        c.img_type = ma::ImageCaptureBase::ImageType::Scene;
                        c.img_type_name = "Scene";
                    } else if (tokens_2[1] == "DepthPlanner") {
                        c.img_type_number = 1;
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthPlanner;
                        c.img_type_name = "DepthPlanner";
                    } else if (tokens_2[1] == "DepthPerspective") {
                        c.img_type_number = 2;
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthPerspective;
                        c.img_type_name = "DepthPerspective";
                    } else if (tokens_2[1] == "DepthVis") {
                        c.img_type_number = 3;
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthVis;
                        c.img_type_name = "DepthVis";
                    } else if (tokens_2[1] == "DisparityNormalized") {
                        c.img_type_number = 4;
                        c.img_type = ma::ImageCaptureBase::ImageType::DisparityNormalized;
                        c.img_type_name = "DisparityNormalized";
                    } else if (tokens_2[1] == "Segmentation") {
                        c.img_type_number = 5;
                        c.img_type = ma::ImageCaptureBase::ImageType::Segmentation;
                        c.img_type_name = "Segmentation";
                    } else if (tokens_2[1] == "SurfaceNormals") {
                        c.img_type_number = 6;
                        c.img_type = ma::ImageCaptureBase::ImageType::SurfaceNormals;
                        c.img_type_name = "SurfaceNormals";
                    } else if (tokens_2[1] == "Infrared") {
                        c.img_type_number = 7;
                        c.img_type = ma::ImageCaptureBase::ImageType::Infrared;
                        c.img_type_name = "Infrared";
                    } else {
                        cout << "Error: Unknown image type: " << tokens_2[1] << endl;
                        c.img_type = ma::ImageCaptureBase::ImageType::Scene;
                        c.img_type_name = "Scene";
                    }

                    // Get Image pixel width and height (integers)
                    c.width = std::stoi(tokens_2[2]);
                    c.height = std::stoi(tokens_2[3]);

                    cout << "Adding Camera: " << c << endl;
                    cam_configs_.push_back(c);

                } catch (boost::bad_lexical_cast) {
                    // Parsing whitespace and possibily malformed XML.
                    cout << "Parse Error: Check camera_config's in AirSimSensor.xml" << endl;
                } // end try, catch
            }
        } // end check token_1 size
    } // end for loop: tokens_2 in token_1
} // end parse_camera_configs

void AirSimSensor::init(std::map<std::string, std::string> &params) {
    airsim_ip_ = sc::get<std::string>("airsim_ip", params, "localhost");
    airsim_port_ = sc::get<int>("airsim_port", params, 41451);
    airsim_timeout_s_ = sc::get<int>("airsim_timeout_ms", params, 60);

    data_acquisition_period_ = sc::get<double>("data_acquisition_period", params, 0.1);
    vehicle_name_ = sc::get<std::string>("vehicle_name", params, "robot1");
    lidar_name_ = sc::get<std::string>("lidar_name", params, "lidar1");
    cout << "Vehicle Name: " << vehicle_name_ << endl;
    cout << "Lidar Name: " << lidar_name_ << endl;

    save_airsim_data_ = sc::get<bool>("save_airsim_data", params, "true");
    get_image_data_ = sc::get<bool>("get_image_data", params, "true");
    get_lidar_data_ = sc::get<bool>("get_lidar_data", params, "true");
    if (save_airsim_data_) {
        cout << "Saving camera images and airsim_data.csv of pose to SCRIMMAGE Logs Directory." << endl;
    }
    if (get_image_data_) {
        cout << "Retrieving image data within AirSimSensor::request_images() thread." << endl;
    }
    if (get_lidar_data_) {
        cout << "Retrieving LIDAR data within AirSimSensor::request_images() thread." << endl;
    }
    cout << "Data Acquisition Period = " << data_acquisition_period_ << endl;

    // Get camera configurations
    AirSimSensor::parse_camera_configs(params);

    // Open airsim_data CSV for append (app) and set column headers
    std::string csv_filename = parent_->mp()->log_dir() + "/airsim_data_robot" + std::to_string(parent_->id().id()) + ".csv";
    if (!csv.open_output(csv_filename, std::ios_base::app)) std::cout << "Couldn't create csv file" << endl;
    if (!csv.output_is_open()) cout << "File isn't open. Can't write to CSV" << endl;
    csv.set_column_headers("frame, t, x, y, z, roll, pitch, yaw");

    // Publish to scrimmage
    img_pub_ = advertise("LocalNetwork", "AirSimImages");
    lidar_pub_ = advertise("LocalNetwork", "AirSimLidar");

    // Start the image request thread
    request_images_thread_ = std::thread(&AirSimSensor::request_images, this);

    return;
}

void AirSimSensor::request_images() {
    std::shared_ptr<ma::RpcLibClientBase> img_client =
        std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                     airsim_port_,
                                                     airsim_timeout_s_);
    bool connected = false;
    while (!connected) {
        img_client->confirmConnection();

        // If we haven't been able to connect to AirSim, warn the user
        if (img_client->getConnectionState() !=
            ma::RpcLibClientBase::ConnectionState::Connected) {
            cout << "Warning: Image client not connected to AirSim." << endl;
        } else {
            connected = true;
        }
    }

    running_mutex_.lock();
    bool running = running_;
    running_mutex_.unlock();

    typedef ma::ImageCaptureBase::ImageRequest ImageRequest;
    typedef ma::ImageCaptureBase::ImageResponse ImageResponse;

    // todo, Need to figure out how to pull vehicle/ lidar names from settings.json file
    // Right now we are running AirSim with only 1 Drone

    while (running) {

        // Set up stream of camera images and lidar data to be published to scrimmage messages.

        // LIDAR data is the same for each corresponding image group requested
        // You can also get segmented objects from lidar data in AirSim, see RpcLibClientBase::simGetLidarSegmentation
        // Get Lidar Data
        if (get_lidar_data_) {
            AirSimLidarType l;
            l.vehicle_name = vehicle_name_;
            l.lidar_name = lidar_name_;
            // lidar_data contains point_cloud, timestamp, and pose (position and orientation)
            // See AirLib::RpcAdaptorsBase
            l.lidar_data = img_client->getLidarData(lidar_name_, vehicle_name_);
            // Get pose of vehicle
            ma::Pose vehicle_pose = img_client->simGetVehiclePose(vehicle_name_);
            // AirSim vehicle pose is in NED, but scrimmage is in ENU so convert
            // Convert position to ENU: Switch X and Y and negate Z
            Eigen::Matrix<float, 3, 1> vehicle_position_ENU;
            vehicle_position_ENU.x() = vehicle_pose.position.y();
            vehicle_position_ENU.y() = vehicle_pose.position.x();
            vehicle_position_ENU.z() = -1 * vehicle_pose.position.z();
            // Convert the orientation to ENU
            Eigen::Quaternion<float> NED_quat = vehicle_pose.orientation;
            // Rotate, order matters
            double xTo = -180 * (M_PI / 180); // -PI rotation about X
            double zTo = 90 * (M_PI / 180); // PI/2 rotation about Z (Up)
            Eigen::Quaternion<float> vehicle_orientation_ENU;
            vehicle_orientation_ENU = Eigen::AngleAxis<float>(xTo, Eigen::Vector3f::UnitX()) * NED_quat; // -PI rotation about X
            vehicle_orientation_ENU = Eigen::AngleAxis<float>(zTo, Eigen::Vector3f::UnitZ()) * vehicle_orientation_ENU; // PI/2 rotation about Z (Up)
            // Bring vehicle pose in relation to ENU, World into Eigen
            Eigen::Translation3f translation_trans_vehicle(vehicle_position_ENU);
            Eigen::Quaternionf rotation_quat_vehicle(vehicle_orientation_ENU);
            Eigen::Isometry3f tf_world_vehicle_ENU(translation_trans_vehicle * rotation_quat_vehicle);

            // AirSim gives pose of lidar in relation to the world frame
            // Pose in lidar_data_ is in NED, but scrimmage is in ENU so convert
            // Convert position to ENU: Switch X and Y and negate Z
            Eigen::Matrix<float, 3, 1> lidar_position_world_ENU;
            lidar_position_world_ENU.x() = l.lidar_data.pose.position.y();
            lidar_position_world_ENU.y() = l.lidar_data.pose.position.x();
            lidar_position_world_ENU.z() = -1 * l.lidar_data.pose.position.z();
            // Convert the orientation to ENU
            Eigen::Quaternion<float> NED_quat_lidar = l.lidar_data.pose.orientation;
            // Rotate, order matters
            // double xTo = -180 * (M_PI / 180); // -PI rotation about X
            // double zTo = 90 * (M_PI / 180); // PI/2 rotation about Z (Up)
            Eigen::Quaternion<float> lidar_orientation_world_ENU;
            lidar_orientation_world_ENU = Eigen::AngleAxis<float>(xTo, Eigen::Vector3f::UnitX()) * NED_quat_lidar; // -PI rotation about X
            lidar_orientation_world_ENU = Eigen::AngleAxis<float>(zTo, Eigen::Vector3f::UnitZ()) * lidar_orientation_world_ENU; // PI/2 rotation about Z (Up)
            // Place pose of lidar in ENU, world frame into Eigen
            Eigen::Translation3f translation_trans_lidar(lidar_position_world_ENU);
            Eigen::Quaternionf rotation_quat_lidar(lidar_orientation_world_ENU);
            Eigen::Isometry3f tf_world_lidar_ENU(translation_trans_lidar * rotation_quat_lidar);

            // Get pose of lidar in ENU, vehicle frame using Eigen
            Eigen::Isometry3f tf_vehicle_lidar_ENU(tf_world_vehicle_ENU.inverse() * tf_world_lidar_ENU);

            // Place pose of lidar in ENU, vehicle frame in lidar message
            l.lidar_position_ENU.x() = tf_vehicle_lidar_ENU.translation().x();
            l.lidar_position_ENU.y() = tf_vehicle_lidar_ENU.translation().y();
            l.lidar_position_ENU.z() = tf_vehicle_lidar_ENU.translation().z();
            Eigen::Quaternionf tf_vehicle_lidar_ENU_rotation(tf_vehicle_lidar_ENU.rotation());
            l.lidar_orientation_ENU.w() = tf_vehicle_lidar_ENU_rotation.w();
            l.lidar_orientation_ENU.x() = tf_vehicle_lidar_ENU_rotation.x();
            l.lidar_orientation_ENU.y() = tf_vehicle_lidar_ENU_rotation.y();
            l.lidar_orientation_ENU.z() = tf_vehicle_lidar_ENU_rotation.z();
            // Place Vehicle pose in ENU, World frame in message for use in creating the transform tree in ROSAirSim
            l.vehicle_pose = tf_world_vehicle_ENU;

            if (l.lidar_data.point_cloud.size() > 3) {
                auto lidar_msg = std::make_shared<sc::Message<AirSimLidarType>>();
                lidar_msg->data = l;

                new_lidar_mutex_.lock();
                new_lidar_ = true;
                new_lidar_mutex_.unlock();

                lidar_msg_mutex_.lock();
                lidar_msg_ = lidar_msg;
                lidar_msg_mutex_.unlock();
            }
        }

        if (get_image_data_) {
            // Make the Requests Vector for getting images
            vector<ImageRequest> requests;
            for (CameraConfig c : cam_configs_) {
                // Depth Images (Depth Perspective and Depth Planner) come in as 1 channel float arrays
                ImageRequest request;
                if (c.img_type_name == "DepthPerspective" || c.img_type_name == "DepthPlanner") {
                    request = {ImageRequest(c.cam_name, c.img_type, true, false)};
                } else {
                    // All Other Image types come in as RGB = 3 channel uint8 matrix
                    request = {ImageRequest(c.cam_name, c.img_type, false, false)};
                }
                requests.push_back(request);
            }

            // Get Images
            const std::vector<ImageResponse>& response_vector  = img_client->simGetImages(requests, vehicle_name_);
            // reinitiate vehicle pose to ensure it is up to date with camera pose
            ma::Pose vehicle_pose = img_client->simGetVehiclePose(vehicle_name_);
            // AirSim vehicle pose is in NED, but scrimmage is in ENU so convert
            // Convert position to ENU: Switch X and Y and negate Z
            Eigen::Matrix<float, 3, 1> vehicle_position_ENU;
            vehicle_position_ENU.x() = vehicle_pose.position.y();
            vehicle_position_ENU.y() = vehicle_pose.position.x();
            vehicle_position_ENU.z() = -1 * vehicle_pose.position.z();
            // Convert the orientation to ENU
            Eigen::Quaternion<float> NED_quat_vehicle = vehicle_pose.orientation;
            // Rotate, order matters
            double xTo = -180 * (M_PI / 180); // -PI rotation about X
            double zTo = 90 * (M_PI / 180); // PI/2 rotation about Z (Up)
            Eigen::Quaternion<float> vehicle_orientation_ENU;
            vehicle_orientation_ENU = Eigen::AngleAxis<float>(xTo, Eigen::Vector3f::UnitX()) * NED_quat_vehicle; // -PI rotation about X
            vehicle_orientation_ENU = Eigen::AngleAxis<float>(zTo, Eigen::Vector3f::UnitZ()) * vehicle_orientation_ENU; // PI/2 rotation about Z (Up)
            // Bring vehicle pose in relation to ENU, World into Eigen
            Eigen::Translation3f translation_trans_vehicle(vehicle_position_ENU);
            Eigen::Quaternionf rotation_quat_vehicle(vehicle_orientation_ENU);
            Eigen::Isometry3f tf_world_vehicle_ENU(translation_trans_vehicle * rotation_quat_vehicle);

            // If response vector contains new data, create img_msg, transfer data into message and publish
            // cout << response_vector.size() << endl;
            if (response_vector.size() > 0) {
                auto im_msg = std::make_shared<sc::Message<std::vector<AirSimImageType>>>();

                new_image_mutex_.lock();
                new_image_ = true;
                new_image_mutex_.unlock();

                std::vector<AirSimImageType> image_type_vec;
                for (ImageResponse response : response_vector) {
                    AirSimImageType a;
                    a.vehicle_name = vehicle_name_;
                    // Get Camera Config
                    for (CameraConfig c : cam_configs_) {
                        if (c.img_type == response.image_type && c.cam_name == response.camera_name) {
                            a.camera_config = c;
                            break;
                        }
                    }
                    // AirSim gives pose of camera in relation to the world frame
                    // Pose in response is in NED, but scrimmage is in ENU so convert
                    // Convert position to ENU: Switch X and Y and negate Z
                    Eigen::Matrix<float, 3, 1> camera_position_world_ENU;
                    camera_position_world_ENU.x() = response.camera_position.y();
                    camera_position_world_ENU.y() = response.camera_position.x();
                    camera_position_world_ENU.z() = -1 * response.camera_position.z();
                    // Convert the orientation to ENU
                    Eigen::Quaternion<float> NED_quat_camera = response.camera_orientation;
                    // Rotate, order matters
                    double xTo = -180 * (M_PI / 180); // -PI rotation about X
                    double zTo = 90 * (M_PI / 180); // PI/2 rotation about Z (Up)
                    Eigen::Quaternion<float> camera_orientation_world_ENU;
                    camera_orientation_world_ENU = Eigen::AngleAxis<float>(xTo, Eigen::Vector3f::UnitX()) * NED_quat_camera; // -PI rotation about X
                    camera_orientation_world_ENU = Eigen::AngleAxis<float>(zTo, Eigen::Vector3f::UnitZ()) * camera_orientation_world_ENU; // PI/2 rotation about Z (Up)
                    // Place pose of camera in ENU, world frame into Eigen
                    Eigen::Translation3f translation_trans_camera(camera_position_world_ENU);
                    Eigen::Quaternionf rotation_quat_camera(camera_orientation_world_ENU);
                    Eigen::Isometry3f tf_world_camera_ENU(translation_trans_camera * rotation_quat_camera);

                    // Get pose of lidar in ENU, vehicle frame using Eigen
                    Eigen::Isometry3f tf_vehicle_camera_ENU(tf_world_vehicle_ENU.inverse() * tf_world_camera_ENU);

                    // Place pose of lidar in ENU, vehicle frame in lidar message
                    a.camera_config.cam_position_ENU.x() = tf_vehicle_camera_ENU.translation().x();
                    a.camera_config.cam_position_ENU.y() = tf_vehicle_camera_ENU.translation().y();
                    a.camera_config.cam_position_ENU.z() = tf_vehicle_camera_ENU.translation().z();
                    Eigen::Quaternionf tf_vehicle_camera_ENU_rotation(tf_vehicle_camera_ENU.rotation());
                    a.camera_config.cam_orientation_ENU.w() = tf_vehicle_camera_ENU_rotation.w();
                    a.camera_config.cam_orientation_ENU.x() = tf_vehicle_camera_ENU_rotation.x();
                    a.camera_config.cam_orientation_ENU.y() = tf_vehicle_camera_ENU_rotation.y();
                    a.camera_config.cam_orientation_ENU.z() = tf_vehicle_camera_ENU_rotation.z();

                    // Depth Images (Depth Perspective and Depth Planner) come in as 1 channel float arrays
                    a.camera_config.pixels_as_float = response.pixels_as_float;
                    if (a.camera_config.pixels_as_float) {
                        // get uncompressed 32FC1 array bytes
                        auto& im_vec = response.image_data_float;
                        // auto& im_vec = response.at(0).image_data_float;
                        // cout << "response[0] size:" << im_vec.size() << endl;

                        // todo, no memcpy, just set image ptr to underlying data
                        // image = cv::Mat(144, 256, CV_8UC3, static_cast<uint8_t*>(im_vec.data()));
                        cv::Mat img(a.camera_config.height, a.camera_config.width, CV_32FC1);
                        memcpy(img.data, im_vec.data(), im_vec.size() * sizeof(float_t));
                        a.img = std::move(img);
                    } else {
                        // All Other Image types come in as RGB = 3 channel uint8 matrix
                        // get uncompressed rgb array bytes
                        auto& im_vec = response.image_data_uint8;
                        // cout << "response[0] size:" << im_vec.size() << endl;

                        // todo, no memcpy, just set image ptr to underlying data, line below mixes image data from different image types
                        // a.img = cv::Mat(c.height, c.width, CV_8UC3, const_cast<uint8_t*>(im_vec.data()));
                        // a.img = cv::Mat(c.height, c.width, CV_8UC3, const_cast<uint8_t*>(response.at(0).image_data_uint8.data()));

                        // cv::Mat img({c.height, c.width, 3}, CV_8UC3);
                        cv::Mat img(a.camera_config.height, a.camera_config.width, CV_8UC3);
                        memcpy(img.data, im_vec.data(), im_vec.size() * sizeof(uint8_t));
                        a.img = std::move(img);
                        // cv::cvtColor(img, a.img, cv::COLOR_RGB2BGR, 3);

                    } // end if pixels_as_float
                    image_type_vec.push_back(std::move(a));
                    im_msg->data = image_type_vec;
                } // end ImageResponse for loop

                // Place data inside img_msg
                im_msg->data = image_type_vec;

                // Submit message to be published
                img_msg_mutex_.lock();
                img_msg_ = im_msg;
                img_msg_mutex_.unlock();

            } // end response > 0

        } // end if get_image_data

        running_mutex_.lock();
        running = running_;
        running_mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(data_acquisition_period_*1000)));
    } // end while running
} // end AirSimSensor::request_images

bool AirSimSensor::step() {
    ///////////////////////////////////////////////////////////////////////////
    /// Client Connection / Disconnection Handling
    ///////////////////////////////////////////////////////////////////////////

    // If we aren't connected to AirSim, re-establish connection.
    if (!client_connected_) {
        cout << "Connecting to AirSim: ip " << airsim_ip_ << ", port " << airsim_port_ << endl;
        sim_client_ = std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                                   airsim_port_,
                                                                   airsim_timeout_s_);

        sim_client_->confirmConnection();

        // If we haven't been able to connect to AirSim, warn the user and return.
        if (sim_client_->getConnectionState() !=
            ma::RpcLibClientBase::ConnectionState::Connected) {
            client_connected_ = false;
            cout << "Warning: not connected to AirSim." << endl;

            // return std::make_shared<sc::MessageBase>();
            return true;
        }
        client_connected_ = true;
        sim_client_->enableApiControl(true, vehicle_name_);
    }

    ///////////////////////////////////////////////////////////////////////////
    /// Set pose of vehicle in AirSim
    ///////////////////////////////////////////////////////////////////////////

    // Setup state information for AirSim
    sc::StatePtr &state = parent_->state_truth();
    // convert from ENU to NED frame
    ma::Vector3r pos(state->pos()(1), state->pos()(0), -state->pos()(2));

    enu_to_ned_yaw_.set_angle(ang::rad2deg(state->quat().yaw()));
    double airsim_yaw_rad = ang::deg2rad(enu_to_ned_yaw_.angle());

    // pitch, roll, yaw
    // note, the negative pitch and yaw are required because of the wsu coordinate frame
    ma::Quaternionr qd = ma::VectorMath::toQuaternion(-state->quat().pitch(),
                                                      state->quat().roll(),
                                                      airsim_yaw_rad);

    // Send state information to AirSim
    sim_client_->simSetVehiclePose(ma::Pose(pos, qd), true, vehicle_name_);

    // Get the camera images from the other thread
    if (get_image_data_) {
        sc::MessagePtr<std::vector<AirSimImageType>> im_msg;
        bool new_image = false;

        new_image_mutex_.lock();
        new_image = new_image_;
        new_image_ = false;
        new_image_mutex_.unlock();

        img_msg_mutex_.lock();
        im_msg = img_msg_;
        img_msg_mutex_.unlock();

        // If image is new, publish
        if (new_image) {
            if (save_airsim_data_) {
                AirSimSensor::save_data(im_msg, state, airsim_frame_num_);
            }
            img_pub_->publish(im_msg);
        }
    }

    if (get_lidar_data_) {
        sc::MessagePtr<AirSimLidarType> lidar_msg;
        bool new_lidar = false;

        new_lidar_mutex_.lock();
        new_lidar = new_lidar_;
        new_lidar_ = false;
        new_lidar_mutex_.unlock();

        lidar_msg_mutex_.lock();
        lidar_msg = lidar_msg_;
        lidar_msg_mutex_.unlock();

        // If lidar is new, publish
        if (new_lidar) {
            lidar_pub_->publish(lidar_msg);
        }
    }

    airsim_frame_num_++;

    return true;
}

bool AirSimSensor::save_data(MessagePtr<std::vector<AirSimImageType>>& im_msg, sc::StatePtr& state, int frame_num) {
    // Get timestamp
    double time_now = time_->t();

    // TODO: Saving 3 images + a csv makes the simulation run slower, maybe need to thread
    for (AirSimImageType d : im_msg->data) {
        // Create Vehicle Directory
        std::string vehicle_dir = parent_->mp()->log_dir() + "/" + vehicle_name_ + "/";
        boost::filesystem::create_directory(vehicle_dir);
        // Create Image type directory and image file name
        std::string img_type_dir = vehicle_dir + d.camera_config.img_type_name + "/";
        boost::filesystem::create_directory(img_type_dir);
        std::string img_filename = img_type_dir + std::to_string(frame_num) + ".png";

        // write image
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        // 9 is greatest compression, most process time. 0 is least compression.
        compression_params.push_back(0);
        cv::imwrite(img_filename, d.img, compression_params);
    }

    // Write the CSV file to the root log directory file name = airsim_data.csv
    if (!csv.output_is_open()) {
            cout << "File isn't open. Can't append to CSV" << endl;
            }
    csv.append(sc::CSV::Pairs{
    {"frame", frame_num},
    {"t", time_now},
    {"x", state->pos()(0)},
    {"y", state->pos()(1)},
    {"z", state->pos()(2)},
    {"roll", state->quat().roll()},
    {"pitch", state->quat().pitch()},
    {"yaw", state->quat().yaw()}}, true, true);

    return true;
}

void AirSimSensor::close(double t) {
    csv.close_output();

    // Safely join with the image client thread
    running_mutex_.lock();
    running_ = false;
    running_mutex_.unlock();

    request_images_thread_.join();
}

} // namespace sensor
} // namespace scrimmage
