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
 * @brief Requests AirSim data over RPC and publishes it as SCRIMMAGE messages.
 * @section Requests AirSim data over RPC and publishes it as SCRIMMAGE messages.
 * Requests AirSim data over RPC and publishes it as SCRIMMAGE messages.
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
    // [VehicleName=robot1 CameraName=front_center ImageTypeName=Scene Width=256 Height=144]
    std::string camera_config = sc::get<std::string>("camera_config", params, "");
    std::vector<std::string> tokens_1;
    boost::split(tokens_1, camera_config, boost::is_any_of("[]"));
    for (std::string &t_1 : tokens_1) {
        if (t_1.length() > 0) {
            std::vector<std::string> tokens_2;
            boost::split(tokens_2, t_1, boost::is_any_of(" ,"));

            if ((tokens_2.size() == 6) && (tokens_2[0] == vehicle_name_)) {

                try {
                    CameraConfig c;
                    c.vehicle_name = tokens_2[0];
                    c.cam_name = tokens_2[1];

                    // Get Image Type Name and Number
                    // Scene=0, DepthPlanner=1, DepthPerspective=2, DepthVis=3, DisparityNormalized=4,
                    // Segmentation=5, SurfaceNormals=6, Infrared=7
                    if (tokens_2[2] == "Scene") {
                        c.img_type_number = 0;
                        c.img_type = ma::ImageCaptureBase::ImageType::Scene;
                        c.img_type_name = "Scene";
                    } else if (tokens_2[2] == "DepthPlanner") {
                        c.img_type_number = 1;
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthPlanner;
                        c.img_type_name = "DepthPlanner";
                    } else if (tokens_2[2] == "DepthPerspective") {
                        c.img_type_number = 2;
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthPerspective;
                        c.img_type_name = "DepthPerspective";
                    } else if (tokens_2[2] == "DepthVis") {
                        c.img_type_number = 3;
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthVis;
                        c.img_type_name = "DepthVis";
                    } else if (tokens_2[2] == "DisparityNormalized") {
                        c.img_type_number = 4;
                        c.img_type = ma::ImageCaptureBase::ImageType::DisparityNormalized;
                        c.img_type_name = "DisparityNormalized";
                    } else if (tokens_2[2] == "Segmentation") {
                        c.img_type_number = 5;
                        c.img_type = ma::ImageCaptureBase::ImageType::Segmentation;
                        c.img_type_name = "Segmentation";
                    } else if (tokens_2[2] == "SurfaceNormals") {
                        c.img_type_number = 6;
                        c.img_type = ma::ImageCaptureBase::ImageType::SurfaceNormals;
                        c.img_type_name = "SurfaceNormals";
                    } else if (tokens_2[2] == "Infrared") {
                        c.img_type_number = 7;
                        c.img_type = ma::ImageCaptureBase::ImageType::Infrared;
                        c.img_type_name = "Infrared";
                    } else {
                        cout << "Error: Unknown image type: " << tokens_2[2] << endl;
                        c.img_type = ma::ImageCaptureBase::ImageType::Scene;
                        c.img_type_name = "Scene";
                    }

                    // Get Image pixel width and height (integers)
                    c.width = std::stoi(tokens_2[3]);
                    c.height = std::stoi(tokens_2[4]);
                    c.fov = std::stoi(tokens_2[5]);

                    cout << "[AirSimSensor] Adding camera to Vehicle '" << vehicle_name_ << "' " << c << endl;
                    cam_configs_.push_back(c);

                } catch (boost::bad_lexical_cast) {
                    // Parsing whitespace and possibily malformed XML.
                    cout << "Parse Error: Check camera_config's in AirSimSensor.xml" << endl;
                } // end try, catch
            }
        } // end check token_1 size
    } // end for loop: tokens_2 in token_1
} // end parse_camera_configs

void AirSimSensor::parse_lidar_configs(std::map<std::string, std::string> &params) {
    // Parse the lidar config string.
    // The string is a list of lidar configs from AirSimSensor.xml of the form:
    // [VehicleName LidarName]
    std::string lidar_config = sc::get<std::string>("lidar_config", params, "");
    std::vector<std::string> tokens_1;
    boost::split(tokens_1, lidar_config, boost::is_any_of("[]"));
    for (std::string &t_1 : tokens_1) {
        if (t_1.length() > 0) {
            std::vector<std::string> tokens_2;
            boost::split(tokens_2, t_1, boost::is_any_of(" ,"));

            if ((tokens_2.size() == 2) && (tokens_2[0] == vehicle_name_)) {
                try {
                    cout << "[AirSimSensor] Adding LIDAR sensor '" << tokens_2[1]
                         << "' to Vehicle '" << vehicle_name_ << "'." << endl;
                    lidar_names_.push_back(tokens_2[1]);
                } catch (boost::bad_lexical_cast) {
                    // Parsing whitespace and possibily malformed XML.
                }
            }
        }
    }
}

void AirSimSensor::parse_imu_configs(std::map<std::string, std::string> &params) {
    // Parse the imu config string.
    // The string is a list of imu configs from AirSimSensor.xml of the form:
    // [VehicleName ImuName]
    std::string imu_config = sc::get<std::string>("imu_config", params, "");
    std::vector<std::string> tokens_1;
    boost::split(tokens_1, imu_config, boost::is_any_of("[]"));
    for (std::string &t_1 : tokens_1) {
        if (t_1.length() > 0) {
            std::vector<std::string> tokens_2;
            boost::split(tokens_2, t_1, boost::is_any_of(" ,"));

            if ((tokens_2.size() == 2) && (tokens_2[0] == vehicle_name_)) {
                try {
                    cout << "[AirSimSensor] Adding IMU sensor '" << tokens_2[1]
                         << "' to Vehicle '" << vehicle_name_ << "'." << endl;
                    imu_names_.push_back(tokens_2[1]);
                } catch (boost::bad_lexical_cast) {
                    // Parsing whitespace and possibily malformed XML.
                }
            }
        }
    }
}

void AirSimSensor::init(std::map<std::string, std::string> &params) {
    // RPC / AirSim.xml file
    airsim_ip_ = sc::get<std::string>("airsim_ip", params, "localhost");
    airsim_port_ = sc::get<int>("airsim_port", params, 41451);
    airsim_timeout_s_ = sc::get<int>("airsim_timeout_ms", params, 60);

    // Mission File params
    vehicle_name_ = sc::get<std::string>("vehicle_name", params, "robot1");
    cout << "[AirSimSensor] Vehicle Name: " << vehicle_name_ << endl;
    save_airsim_data_ = sc::get<bool>("save_airsim_data", params, "true");
    if (save_airsim_data_) {
        cout << "[AirSimSensor] Saving camera images and airsim_data.csv of pose to SCRIMMAGE Logs Directory." << endl;
    }
    get_image_data_ = sc::get<bool>("get_image_data", params, "true");
    get_lidar_data_ = sc::get<bool>("get_lidar_data", params, "true");
    get_imu_data_ = sc::get<bool>("get_imu_data", params, "true");
    // data_acquisition_period_ = sc::get<double>("data_acquisition_period", params, 0.1);
    image_acquisition_period_ = sc::get<double>("image_acquisition_period", params, 0.1);
    lidar_acquisition_period_ = sc::get<double>("lidar_acquisition_period", params, 0.1);
    imu_acquisition_period_ = sc::get<double>("imu_acquisition_period", params, 0.1);


    // Open airsim_data CSV for append (app) and set column headers
    std::string csv_filename =
            parent_->mp()->log_dir() + "/airsim_data_robot" + std::to_string(parent_->id().id()) + ".csv";
    if (!csv.open_output(csv_filename, std::ios_base::app)) std::cout << "Couldn't create csv file" << endl;
    if (!csv.output_is_open()) cout << "File isn't open. Can't write to CSV" << endl;
    csv.set_column_headers("frame, t, x, y, z, roll, pitch, yaw");

    //// Publish Images
    if (get_image_data_) {
        cout << "[AirSimSensor] Retrieving image data within AirSimSensor::request_images() thread." << endl;
        cout << "[AirSimSensor] Image Acquisition Period = " << image_acquisition_period_ << endl;
        // Get camera configurations
        AirSimSensor::parse_camera_configs(params);
        img_pub_ = advertise("LocalNetwork", "AirSimImages");
        auto img_msg_ = std::make_shared<sc::Message<std::vector<AirSimImageType>>>();
        // Start the image request thread
        request_images_thread_ = std::thread(&AirSimSensor::request_images, this);
    }

    //// Publish Lidar
    if (get_lidar_data_) {
        cout << "[AirSimSensor] Retrieving LIDAR data within AirSimSensor::request_lidar() thread." << endl;
        cout << "[AirSimSensor] LIDAR Acquisition Period = " << lidar_acquisition_period_ << endl;
        AirSimSensor::parse_lidar_configs(params);
        lidar_pub_ = advertise("LocalNetwork", "AirSimLidar");
        auto lidar_msg_ = std::make_shared<sc::Message<std::vector<AirSimLidarType>>>();
        // Start the lidar request thread
        request_lidar_thread_ = std::thread(&AirSimSensor::request_lidar, this);
    }

    //// Publish Imu
    if (get_imu_data_) {
        cout << "[AirSimSensor] Retrieving IMU data within AirSimSensor::request_imu() thread." << endl;
        cout << "[AirSimSensor] IMU Acquisition Period = " << imu_acquisition_period_ << endl;
        AirSimSensor::parse_imu_configs(params);
        imu_pub_ = advertise("LocalNetwork", "AirSimImu");
        auto imu_msg_ = std::make_shared<sc::Message<std::vector<AirSimImuType>>>();
        // Start the image request thread
        request_imu_thread_ = std::thread(&AirSimSensor::request_imu, this);
    }
}

void AirSimSensor::request_images() {
    std::shared_ptr<ma::RpcLibClientBase> img_client =
        std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                     airsim_port_,
                                                     airsim_timeout_s_);
    // cout << vehicle_name_ << " Image client waiting for Unreal/AirSim connection" << endl;
    for (int i = 0; i < 11; i++) {
        if (img_client->getConnectionState() != ma::RpcLibClientBase::ConnectionState::Connected) {
            // cout << "X" << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            // If we haven't been able to connect to AirSim, warn the user and return
            if (i == 10) {
                cout << "\n[AirSimSensor] Warning: Image client could not connect to AirSim." << endl;
                cout << "[AirSimSensor] Warning: Please start Unreal/AirSim before the SCRIMMAGE mission.\n" << endl;
                return;
            }
            std::shared_ptr<ma::RpcLibClientBase> img_client =
                    std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                                 airsim_port_,
                                                                 airsim_timeout_s_);
        } else {
            // cout << vehicle_name_ << " Image Client Connected" << endl;
            break;
        }
    }

    running_mutex_.lock();
    bool running = running_;
    running_mutex_.unlock();

    typedef ma::ImageCaptureBase::ImageRequest ImageRequest;
    typedef ma::ImageCaptureBase::ImageResponse ImageResponse;

    std::chrono::high_resolution_clock::time_point t_start, t_end;
    // std::chrono::high_resolution_clock::time_point r_start, r_end;

    while (running) {
        // Set up stream of camera images to be published to scrimmage messages.
        t_start = std::chrono::high_resolution_clock::now();
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
        // request vehicle pose to ensure it is up to date with camera pose
        ma::Pose vehicle_pose_camera = img_client->simGetVehiclePose(vehicle_name_);
        // AirSim vehicle pose is in NED, but scrimmage is in ENU so convert
        // Convert position to ENU: Switch X and Y and negate Z

        // If response vector contains new data, create img_msg, transfer data into message and publish
        // cout << response_vector.size() << endl;
        if (response_vector.size() > 0) {
            auto im_msg = std::make_shared<sc::Message<std::vector<AirSimImageType>>>();
            // sc::MessagePtr<std::vector<AirSimImageType>> im_msg;

            for (ImageResponse response : response_vector) {
                CameraConfig response_cam_config;
                for (CameraConfig c : cam_configs_) {
                    if (c.img_type == response.image_type && c.cam_name == response.camera_name) {
                        response_cam_config = c;
                        break;
                    }
                }
                // Create AirSimImageType variable
                AirSimImageType a;
                a.camera_config = response_cam_config;
                a.vehicle_name = vehicle_name_;

                // AirSim gives pose of vehicle in relation to the world frame in NED
                Eigen::Translation3f translation_trans_vehicle(vehicle_pose_camera.position);
                Eigen::Quaternionf rotation_quat_vehicle(vehicle_pose_camera.orientation);
                Eigen::Isometry3f tf_world_vehicle_NED(translation_trans_vehicle * rotation_quat_vehicle);
                a.vehicle_pose_world_NED = tf_world_vehicle_NED;
                // AirSim gives pose of camera in relation to the world frame in NED
                Eigen::Translation3f translation_trans_camera(response.camera_position);
                Eigen::Quaternionf rotation_quat_camera(response.camera_orientation);
                Eigen::Isometry3f tf_world_camera_NED(translation_trans_camera * rotation_quat_camera);
                a.camera_pose_world_NED = tf_world_camera_NED;

                // Depth Images (Depth Perspective and Depth Planner) come in as 1 channel float arrays
                a.camera_config.pixels_as_float = response.pixels_as_float;
                if (a.camera_config.pixels_as_float) {
                    // get uncompressed 32FC1 array bytes
                    a.img = cv::Mat(a.camera_config.height, a.camera_config.width, CV_32FC1);
                    memcpy(a.img.data, response.image_data_float.data(), response.image_data_float.size() * sizeof(float_t));

                } else {
                    // All Other Image types come in as RGB = 3 channel uint8 matrix
                    // However old linux asset environments give 4 channels

                    // If image has 4 channels
                    if (((int)response.image_data_uint8.size()) > (a.camera_config.height * a.camera_config.width * 3)) {
                        a.img = cv::Mat(a.camera_config.height, a.camera_config.width, CV_8UC4);
                        memcpy(a.img.data, response.image_data_uint8.data(),response.image_data_uint8.size() * sizeof(uint8_t));
                    } else {
                        // image has 3 channels
                        a.img = cv::Mat(a.camera_config.height, a.camera_config.width, CV_8UC3);
                        memcpy(a.img.data, response.image_data_uint8.data(),response.image_data_uint8.size() * sizeof(uint8_t));
                    }
                } // end if pixels_as_float
                im_msg->data.push_back(a);
            } // end ImageResponse for loop

            // Submit message to be published
            new_image_mutex_.lock();
            new_image_ = true;
            new_image_mutex_.unlock();

            img_msg_mutex_.lock();
            img_msg_ = im_msg;
            img_msg_mutex_.unlock();

        } // end response > 0

        running_mutex_.lock();
        running = running_;
        running_mutex_.unlock();
        // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(data_acquisition_period_*1000)));
        t_end = std::chrono::high_resolution_clock::now();
        double t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        t_elapsed = (image_acquisition_period_*1000) - t_elapsed;
        if (t_elapsed > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_elapsed)));
        }
        t_end = std::chrono::high_resolution_clock::now();
        // double image_period_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
        // cout << "avg image period: " << image_period_ << "s" << endl;

    } // end while running
} // end AirSimSensor::request_images

void AirSimSensor::request_lidar() {
    std::shared_ptr<ma::RpcLibClientBase> lidar_client =
            std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                         airsim_port_,
                                                         airsim_timeout_s_);

    // cout << vehicle_name_ << " LIDAR client waiting for Unreal/AirSim connection" << endl;
    for (int i = 0; i < 11; i++) {
        if (lidar_client->getConnectionState() != ma::RpcLibClientBase::ConnectionState::Connected) {
            // cout << "X" << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            // If we haven't been able to connect to AirSim, warn the user and return
            if (i == 10) {
                cout << "\n[AirSimSensor] Warning: LIDAR client could not connect to AirSim." << endl;
                cout << "[AirSimSensor] Warning: Please start Unreal/AirSim before the SCRIMMAGE mission.\n" << endl;
                return;
            }
            std::shared_ptr<ma::RpcLibClientBase> lidar_client =
                    std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                                 airsim_port_,
                                                                 airsim_timeout_s_);
        } else {
            // cout << vehicle_name_ << " LIDAR Client Connected" << endl;
            break;
        }
    }

    running_mutex_.lock();
    bool running = running_;
    running_mutex_.unlock();

    std::chrono::high_resolution_clock::time_point t_start, t_end;
    while (running) {
        t_start = std::chrono::high_resolution_clock::now();
        auto lidar_msg = std::make_shared<sc::Message<std::vector<AirSimLidarType>>>();
        bool new_lidar = false;

        for (const std::string& lidar_name : lidar_names_) {
            AirSimLidarType l;
            l.vehicle_name = vehicle_name_;
            l.lidar_name = lidar_name;
            l.lidar_data = lidar_client->getLidarData(lidar_name, vehicle_name_);

            // Get pose of vehicle
            ma::Pose vehicle_pose = lidar_client->simGetVehiclePose(vehicle_name_);
            Eigen::Translation3f translation_trans_vehicle(vehicle_pose.position);
            Eigen::Quaternionf rotation_quat_vehicle(vehicle_pose.orientation);
            Eigen::Isometry3f tf_world_vehicle_NED(translation_trans_vehicle * rotation_quat_vehicle);
            l.vehicle_pose_world_NED = tf_world_vehicle_NED;
            // Get pose of LIDAR
            Eigen::Translation3f translation_trans_lidar(l.lidar_data.pose.position);
            Eigen::Quaternionf rotation_quat_lidar(l.lidar_data.pose.orientation);
            Eigen::Isometry3f tf_world_lidar_NED(translation_trans_lidar * rotation_quat_lidar);
            l.lidar_pose_world_NED = tf_world_lidar_NED;

            if (l.lidar_data.point_cloud.size() > 3) {
                new_lidar = true;
                lidar_msg->data.push_back(l);
            }
        }

        lidar_msg_mutex_.lock();
        lidar_msg_ = lidar_msg;
        lidar_msg_mutex_.unlock();

        new_lidar_mutex_.lock();
        new_lidar_ = new_lidar;
        new_lidar_mutex_.unlock();

        running_mutex_.lock();
        running = running_;
        running_mutex_.unlock();

        t_end = std::chrono::high_resolution_clock::now();
        double t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        t_elapsed = (lidar_acquisition_period_*1000) - t_elapsed;
        if (t_elapsed > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_elapsed)));
        }
        t_end = std::chrono::high_resolution_clock::now();
        // double lidar_period_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
        // cout << "avg lidar period: " << lidar_period_ << "s" << endl;
    }
}

void AirSimSensor::request_imu() {
    std::shared_ptr<ma::RpcLibClientBase> imu_client =
            std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                         airsim_port_,
                                                         airsim_timeout_s_);
    // cout << vehicle_name_ << " IMU waiting for Unreal/AirSim connection" << endl;
    for (int i = 0; i < 11; i++) {
        if (imu_client->getConnectionState() != ma::RpcLibClientBase::ConnectionState::Connected) {
            // cout << "X" << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            // If we haven't been able to connect to AirSim, warn the user and return
            if (i == 10) {
                cout << "\n[AirSimSensor] Warning: IMU client could not connect to AirSim." << endl;
                cout << "[AirSimSensor] Warning: Please start Unreal/AirSim before the SCRIMMAGE mission.\n" << endl;
                return;
            }
            std::shared_ptr<ma::RpcLibClientBase> imu_client =
                    std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                                 airsim_port_,
                                                                 airsim_timeout_s_);
        } else {
            // cout << vehicle_name_ << " IMU Client Connected" << endl;
            break;
        }
    }

    running_mutex_.lock();
    bool running = running_;
    running_mutex_.unlock();

    std::chrono::high_resolution_clock::time_point t_start, t_end;
    while (running) {
        t_start = std::chrono::high_resolution_clock::now();
        auto imu_msg = std::make_shared<sc::Message<std::vector<AirSimImuType>>>();

        bool new_imu = false;
        for (const auto& imu_name : imu_names_) {
            // Get IMU Data
            AirSimImuType i;
            i.vehicle_name = vehicle_name_;
            i.imu_name = imu_name;
            i.imu_data = imu_client->getImuData(imu_name, vehicle_name_);
            // Get pose of vehicle
            ma::Pose vehicle_pose = imu_client->simGetVehiclePose(vehicle_name_);
            Eigen::Translation3f translation_trans_vehicle(vehicle_pose.position);
            Eigen::Quaternionf rotation_quat_vehicle(vehicle_pose.orientation);
            Eigen::Isometry3f tf_world_vehicle_NED(translation_trans_vehicle * rotation_quat_vehicle);
            i.vehicle_pose_world_NED = tf_world_vehicle_NED;
            // Get pose of IMU
            // AirLib API does not give IMU position so use position for center of vehicle
            Eigen::Translation3f translation_trans_imu(vehicle_pose.position);
            Eigen::Quaternionf rotation_quat_imu(i.imu_data.orientation);
            Eigen::Isometry3f tf_world_imu_NED(translation_trans_imu * rotation_quat_imu);
            i.imu_pose_world_NED = tf_world_imu_NED;
            imu_msg->data.push_back(i);
            new_imu = true;
        }

        imu_msg_mutex_.lock();
        imu_msg_ = imu_msg;
        imu_msg_mutex_.unlock();

        new_imu_mutex_.lock();
        new_imu_ = new_imu;
        new_imu_mutex_.unlock();

        running_mutex_.lock();
        running = running_;
        running_mutex_.unlock();

        t_end = std::chrono::high_resolution_clock::now();
        double t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        t_elapsed = (imu_acquisition_period_*1000) - t_elapsed;
        if (t_elapsed > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_elapsed)));
        }
        t_end = std::chrono::high_resolution_clock::now();
        // double imu_period_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
        // cout << "avg imu period: " << imu_period_ << "s" << endl;
    }
}

bool AirSimSensor::step() {
    ///////////////////////////////////////////////////////////////////////////
    /// Client Connection / Disconnection Handling
    ///////////////////////////////////////////////////////////////////////////

    if (!client_connected_) {
        cout << vehicle_name_ << " Sim Client waiting for Unreal/AirSim connection - " << endl;
        sim_client_ = std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                                   airsim_port_,
                                                                   airsim_timeout_s_);

        // sim_client_->confirmConnection(); -- don't use infinite while loop
        for (int i = 0; i < 11; i++) {
            if (sim_client_->getConnectionState() != ma::RpcLibClientBase::ConnectionState::Connected) {
                cout << "X" << std::flush;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // If we haven't been able to connect to AirSim, warn the user and return.
                if (i == 10) {
                    cout << "\n[AirSimSensor] Warning: Sim client could not connect to AirSim." << endl;
                    cout << "[AirSimSensor] Warning: Please start Unreal/AirSim before the SCRIMMAGE mission.\n" << endl;
                    return false;
                }
                std::shared_ptr<ma::RpcLibClientBase> imu_client =
                        std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                                     airsim_port_,
                                                                     airsim_timeout_s_);
            } else {
                client_connected_ = true;
                cout << "[AirSimSensor] Sim Client for " << vehicle_name_ << " connected to AirSim: ip " << airsim_ip_ << ", port " << airsim_port_ << endl;
                break;
            }
        }

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
        // sc::MessagePtr<std::vector<AirSimImageType>> im_msg_step;
        // auto im_msg_step = std::make_shared<sc::Message<std::vector<AirSimImageType>>>(); // NOLINT

        new_image_mutex_.lock();
        bool new_image = new_image_;
        new_image_ = false;
        new_image_mutex_.unlock();

        img_msg_mutex_.lock();
        // im_msg_step = img_msg_;
        std::shared_ptr<sc::Message<std::vector<AirSimImageType>>> im_msg_step = img_msg_;
        // sc::MessagePtr<std::vector<AirSimImageType>> im_msg_step = img_msg_;
        img_msg_mutex_.unlock();

        // If image is new, publish
        if (new_image) {
            if (save_airsim_data_) {
                AirSimSensor::save_data(im_msg_step, state, airsim_frame_num_);
            }
            img_pub_->publish(im_msg_step);
        }
    }

    if (get_lidar_data_) {
        // sc::MessagePtr<AirSimLidarType> lidar_msg;
        // auto lidar_msg_step = std::make_shared<sc::Message<std::vector<AirSimLidarType>>>(); // NOLINT

        new_lidar_mutex_.lock();
        bool new_lidar = new_lidar_;
        new_lidar_ = false;
        new_lidar_mutex_.unlock();

        lidar_msg_mutex_.lock();
        std::shared_ptr<sc::Message<std::vector<AirSimLidarType>>> lidar_msg_step = lidar_msg_;
        // lidar_msg_step = lidar_msg_; // NOLINT
        lidar_msg_mutex_.unlock();

        // If lidar is new, publish
        if (new_lidar) {
            lidar_pub_->publish(lidar_msg_step);
        }
    }

    if (get_imu_data_) {
        // auto imu_msg_step = std::make_shared<sc::Message<std::vector<AirSimImuType>>>(); // NOLINT

        new_imu_mutex_.lock();
        bool new_imu = new_imu_;
        new_imu_ = false;
        new_imu_mutex_.unlock();

        imu_msg_mutex_.lock();
        std::shared_ptr<sc::Message<std::vector<AirSimImuType>>> imu_msg_step = imu_msg_;
        // imu_msg_step = imu_msg_;  // NOLINT
        imu_msg_mutex_.unlock();

        // If lidar is new, publish
        if (new_imu) {
            imu_pub_->publish(imu_msg_step);
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
            cout << "[AirSimSensor] File isn't open. Can't append to CSV" << endl;
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
    if (save_airsim_data_) {
        csv.close_output();
    }

    // Safely join with the sensor threads
    running_mutex_.lock();
    running_ = false;
    running_mutex_.unlock();

    request_images_thread_.join();
    request_lidar_thread_.join();
    request_imu_thread_.join();
}

} // namespace sensor
} // namespace scrimmage
